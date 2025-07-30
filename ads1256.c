#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>  // For usleep_range
#include <linux/device.h>  // For device APIs, class_find_device
#include <linux/gpio/consumer.h>  // For GPIO handling
#include <linux/of.h>            // For device tree parsing
#include <linux/cdev.h>          // For cdev operations

// Define constants
#define ADS1256_NAME "ads1256"
#define ADS1256_DRDY_TIMEOUT 33000  // microseconds (33 ms for 30 SPS)

// ADS1256 Commands
#define ADS1256_CMD_RESET   0xFE  // Reset
#define ADS1256_CMD_START   0x08  // Start/Sync (start conversions)
#define ADS1256_CMD_STOP    0x0A  // Stop conversions
#define ADS1256_CMD_RDATA   0x01  // Read Data
#define ADS1256_CMD_RREG    0x10  // Read Register
#define ADS1256_CMD_WREG    0x50  // Write Register

// ADS1256 Register Addresses
#define ADS1256_REG_STATUS  0x00  // Status Register
#define ADS1256_REG_MUX     0x01  // MUX Register (channel selection)
#define ADS1256_REG_ADCON   0x02  // A/D Control Register (PGA, clock)
#define ADS1256_REG_DRATE   0x03  // Data Rate Register

// Default Configuration Values
#define ADS1256_MUX_DEFAULT 0x01  // Select AIN0 (channel 0) as input
#define ADS1256_ADCON_DEFAULT 0x20  // PGA = 1 (gain 1), clock = internal
#define ADS1256_DRATE_DEFAULT 0x03  // Data rate = 30 SPS (samples per second)

// Driver data structure
struct ads1256_data {
    struct spi_device *spi;
    struct mutex lock;
    struct gpio_desc *drdy_gpio;  // GPIO descriptor for DRDY (keeping this)
    u8 tx_buf[4];
    u8 rx_buf[4];
};

static int ads1256_major;
static struct class *ads1256_class;
static struct device *ads1256_device;
static struct cdev ads1256_cdev;  // Add cdev for manual management

// Match function for class_find_device to find device by dev_t
static int ads1256_devt_match(struct device *dev, const void *data)
{
    dev_t target = *(dev_t *)data;
    pr_info("Matching dev=%p, devt=0x%u, target=0x%u\n", dev, (unsigned int)dev->devt, (unsigned int)target);
    return dev && dev->devt == target;
}

// Reset the ADS1256
static int ads1256_reset(struct ads1256_data *data) {
    int ret;
    u8 cmd = ADS1256_CMD_RESET;
    struct spi_transfer t = {
        .tx_buf = &cmd,
        .len = 1,
    };
    struct spi_message m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    mutex_lock(&data->lock);
    ret = spi_sync(data->spi, &m);
    if (ret == 0) {
        // Wait after reset (per datasheet, 0.5 ms minimum) using usleep_range for robustness
        usleep_range(500, 1000);  // Sleep 500-1000 us (0.5-1 ms)
        dev_info(&data->spi->dev, "ADS1256 reset successful\n");
    } else {
        dev_err(&data->spi->dev, "ADS1256 reset failed: %d\n", ret);
    }
    mutex_unlock(&data->lock);

    return ret;
}

// Configure the ADS1256
static int ads1256_configure(struct ads1256_data *data) {
    int ret;
    u8 tx_buf[3], rx_buf[3];
    struct spi_transfer t[] = {
        {
            .tx_buf = tx_buf,
            .rx_buf = rx_buf,
            .len = 3,
        },
    };
    struct spi_message m;

    // Set MUX (AIN0 as input, AINCOM as reference)
    tx_buf[0] = ADS1256_CMD_WREG | ADS1256_REG_MUX;
    tx_buf[1] = 0x00;  // Number of registers to write - 1
    tx_buf[2] = ADS1256_MUX_DEFAULT;  // AIN0, AINCOM
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    mutex_lock(&data->lock);
    ret = spi_sync(data->spi, &m);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set MUX: %d\n", ret);
        goto out;
    }

    // Set ADCON (PGA = 1, internal clock)
    tx_buf[0] = ADS1256_CMD_WREG | ADS1256_REG_ADCON;
    tx_buf[1] = 0x00;
    tx_buf[2] = ADS1256_ADCON_DEFAULT;
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    ret = spi_sync(data->spi, &m);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set ADCON: %d\n", ret);
        goto out;
    }

    // Set DRATE (30 SPS)
    tx_buf[0] = ADS1256_CMD_WREG | ADS1256_REG_DRATE;
    tx_buf[1] = 0x00;
    tx_buf[2] = ADS1256_DRATE_DEFAULT;
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    ret = spi_sync(data->spi, &m);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set DRATE: %d\n", ret);
        goto out;
    }

    // Start conversions
    tx_buf[0] = ADS1256_CMD_START;
    t[0].len = 1;
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    ret = spi_sync(data->spi, &m);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to start conversions: %d\n", ret);
        goto out;
    }

    dev_info(&data->spi->dev, "ADS1256 configured successfully\n");
out:
    mutex_unlock(&data->lock);
    return ret;
}

// Read one sample from channel 0, with DRDY polling
static int ads1256_read_sample(struct ads1256_data *data, u32 *value) {
    int ret;
    struct spi_transfer t[] = {
        {
            .tx_buf = data->tx_buf,
            .len = 1,
        },
        {
            .rx_buf = data->rx_buf,
            .len = 3,
        }
    };
    struct spi_message m;
    int timeout = 1000;  // 1 second timeout (adjust as needed)

    // Poll DRDY (active-low, so wait for it to go low)
    while (gpiod_get_value_cansleep(data->drdy_gpio) && timeout--) {
        usleep_range(1000, 2000);  // Wait 1-2 ms per check
    }
    if (timeout <= 0) {
        dev_err(&data->spi->dev, "DRDY timeout waiting for data ready\n");
        return -ETIMEDOUT;
    }
    dev_info(&data->spi->dev, "DRDY asserted, proceeding to read\n");

    // Send RDATA command
    data->tx_buf[0] = ADS1256_CMD_RDATA;
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);

    mutex_lock(&data->lock);
    dev_info(&data->spi->dev, "Sending RDATA command, TX buffer: 0x%02x\n", data->tx_buf[0]);
    ret = spi_sync(data->spi, &m);
    if (ret == 0) {
        // Combine 3 bytes into 24-bit value (two's complement for signed value)
        *value = (data->rx_buf[0] << 16) | (data->rx_buf[1] << 8) | data->rx_buf[2];
        if (*value & 0x800000) {  // Handle negative values (two's complement)
            *value |= 0xFF000000;  // Sign-extend to 32 bits
        }
        dev_info(&data->spi->dev, "Read sample: 0x%08x (raw: 0x%02x 0x%02x 0x%02x)\n",
                 *value, data->rx_buf[0], data->rx_buf[1], data->rx_buf[2]);
    } else {
        dev_err(&data->spi->dev, "SPI sync failed: %d\n", ret);
    }
    mutex_unlock(&data->lock);

    return ret;
}

static ssize_t ads1256_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
    struct ads1256_data *data = filp->private_data;
    u32 sample;
    int ret;

    if (count < sizeof(u32)) {
        dev_err(&data->spi->dev, "Read count too small: %zu < %zu\n", count, sizeof(u32));
        return -EINVAL;
    }

    ret = ads1256_read_sample(data, &sample);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to read sample: %d\n", ret);
        return ret;
    }

    if (copy_to_user(buf, &sample, sizeof(u32))) {
        dev_err(&data->spi->dev, "Failed to copy to user space\n");
        return -EFAULT;
    }

    dev_info(&data->spi->dev, "Successfully read sample 0x%08x to user space\n", sample);
    return sizeof(u32);
}

static int ads1256_open(struct inode *inode, struct file *filp) {
    struct device *dev;
    struct ads1256_data *data;
    struct spi_device *spi;
    dev_t devt;
    int err;

    pr_info("ads1256_open: inode=%p, i_cdev=%p, i_rdev=%u\n", inode, inode->i_cdev, (unsigned int)inode->i_rdev);

    if (!inode->i_cdev) {
        pr_err("Invalid inode or i_cdev in ads1256_open\n");
        return -ENODEV;
    }

    // Try to get the device from the cdev's kobject parent using kernel's container_of_safe
    dev = container_of_safe(inode->i_cdev->kobj.parent, struct device, kobj);
    pr_info("ads1256_open: kobj.parent=%p, dev=%p\n", inode->i_cdev->kobj.parent, dev);

    if (!dev) {
        pr_err("Invalid device in inode, attempting recovery\n");
        // Try to get the device from the device number (dev_t) using class_find_device, with safety checks
        devt = inode->i_rdev;
        if (!ads1256_class) {
            pr_err("ads1256_class is NULL, cannot recover device\n");
            return -ENODEV;
        }
        dev = class_find_device(ads1256_class, NULL, &devt, ads1256_devt_match);
        if (dev) {
            pr_info("ads1256_open: Recovered device from class: dev=%p, devt=0x%u\n", dev, (unsigned int)devt);
        } else {
            pr_err("Failed to recover device from class, devt=0x%u\n", (unsigned int)devt);
            return -ENODEV;
        }
    }

    // First, try to get data from the device
    data = dev_get_drvdata(dev);
    if (!data) {
        pr_err("Failed to get driver data from device\n");
        // Try to get data from the SPI device directly
        spi = to_spi_device(dev);
        if (spi) {
            data = spi_get_drvdata(spi);
            if (!data) {
                pr_err("Failed to get driver data from SPI device\n");
                return -ENODEV;
            }
        } else {
            pr_err("Failed to convert device to SPI device\n");
            return -ENODEV;
        }
    }

    // Verify SPI and DRDY GPIO are valid before proceeding
    if (!data->spi || !data->drdy_gpio) {
        pr_err("Invalid driver data: spi=%p, drdy_gpio=%p\n", data->spi, data->drdy_gpio);
        return -ENODEV;
    }

    err = 0;  // Initialize error code
    filp->private_data = data;
    pr_info("ads1256_open: Successfully opened with data=%p, err=%d\n", data, err);
    return err;  // Return 0 on success, or propagate error
}

static const struct file_operations ads1256_fops = {
    .owner = THIS_MODULE,
    .read = ads1256_read,
    .open = ads1256_open,
};

static int ads1256_probe(struct spi_device *spi) {
    struct ads1256_data *data;
    int ret;
    int attempt = 0;
    const int MAX_ATTEMPTS = 10;  // Increased attempts for robustness

    data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        dev_err(&spi->dev, "Failed to allocate memory for driver data: %d\n", -ENOMEM);
        return -ENOMEM;
    }

    data->spi = spi;
    mutex_init(&data->lock);
    spi_set_drvdata(spi, data);

    // Get DRDY GPIO from device tree (e.g., named "drdy")
    data->drdy_gpio = gpiod_get(&spi->dev, "drdy", GPIOD_IN);
    if (IS_ERR(data->drdy_gpio)) {
        dev_err(&spi->dev, "Failed to get DRDY GPIO: %ld\n", PTR_ERR(data->drdy_gpio));
        ret = PTR_ERR(data->drdy_gpio);
        goto err_free_data;
    }
    dev_info(&spi->dev, "DRDY GPIO retrieved: %p\n", data->drdy_gpio);

    // Force SPI mode 1 (CPOL=0, CPHA=1) as per ADS1256 datasheet
    spi->mode = SPI_MODE_1;
    spi->bits_per_word = 8;
    spi->max_speed_hz = 1920000;  // 1.92 MHz max per ADS1256 datasheet

    dev_info(&spi->dev, "Attempting SPI setup with mode %d, bits_per_word %d, max_speed_hz %d\n",
             spi->mode, spi->bits_per_word, spi->max_speed_hz);

    ret = spi_setup(spi);
    if (ret) {
        dev_err(&spi->dev, "SPI setup with mode 1 failed: %d\n", ret);
        goto err_put_drdy;
    }
    dev_info(&spi->dev, "SPI setup succeeded with mode 1\n");

    // Dynamically allocate a major number and handle conflicts
    do {
        ret = register_chrdev(0, ADS1256_NAME, &ads1256_fops);
        if (ret < 0) {
            dev_err(&spi->dev, "Failed to register chrdev on attempt %d: %d\n", attempt + 1, ret);
            if (ret == -EBUSY && attempt < MAX_ATTEMPTS) {
                msleep(200);  // Increased delay for robustness
                attempt++;
                continue;
            }
            goto err_put_drdy;
        }
        ads1256_major = ret;
        dev_info(&spi->dev, "Registered chrdev with major number %d\n", ads1256_major);
        break;
    } while (attempt < MAX_ATTEMPTS);

    // Manually manage cdev for better control
    cdev_init(&ads1256_cdev, &ads1256_fops);
    ads1256_cdev.owner = THIS_MODULE;
    ret = cdev_add(&ads1256_cdev, MKDEV(ads1256_major, 0), 1);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to add cdev: %d\n", ret);
        if (ret == -EBUSY) {
            dev_err(&spi->dev, "cdev_add failed with EBUSY, checking for conflicts...\n");
            // Try unregistering and retrying
            unregister_chrdev(ads1256_major, ADS1256_NAME);
            msleep(200);
            ret = register_chrdev(0, ADS1256_NAME, &ads1256_fops);
            if (ret < 0) {
                dev_err(&spi->dev, "Failed to re-register chrdev: %d\n", ret);
                goto err_unreg_chrdev;
            }
            ads1256_major = ret;
            dev_info(&spi->dev, "Re-registered chrdev with major number %d\n", ads1256_major);
            ret = cdev_add(&ads1256_cdev, MKDEV(ads1256_major, 0), 1);
            if (ret < 0) {
                dev_err(&spi->dev, "Failed to re-add cdev: %d\n", ret);
                goto err_unreg_chrdev;
            }
        } else {
            goto err_unreg_chrdev;
        }
    }
    dev_info(&spi->dev, "cdev added: major=%d, minor=0, ret=%d\n", ads1256_major, ret);

    ads1256_class = class_create(THIS_MODULE, ADS1256_NAME);
    if (IS_ERR(ads1256_class)) {
        ret = PTR_ERR(ads1256_class);
        dev_err(&spi->dev, "Failed to create class: %d\n", ret);
        goto err_del_cdev;
    }
    dev_info(&spi->dev, "Class created: %p\n", ads1256_class);

    ads1256_device = device_create(ads1256_class, NULL, MKDEV(ads1256_major, 0), data, ADS1256_NAME);
    if (IS_ERR(ads1256_device)) {
        ret = PTR_ERR(ads1256_device);
        dev_err(&spi->dev, "Failed to create device: %d\n", ret);
        goto err_destroy_class;
    }
    dev_info(&spi->dev, "Device created: %p, major=%d, minor=0\n", ads1256_device, ads1256_major);

    // Store the driver data in the device for later retrieval
    dev_set_drvdata(ads1256_device, data);

    // Initialize and configure the ADS1256
    ret = ads1256_reset(data);
    if (ret) {
        dev_err(&spi->dev, "Failed to reset ADS1256: %d\n", ret);
        goto err_destroy_device;
    }
    ret = ads1256_configure(data);
    if (ret) {
        dev_err(&spi->dev, "Failed to configure ADS1256: %d\n", ret);
        goto err_destroy_device;
    }

    dev_info(&spi->dev, "ADS1256 driver initialized relying on SPI controller CS\n");
    dev_info(&spi->dev, "Triggering udev for device node creation\n");
    kobject_uevent(&ads1256_device->kobj, KOBJ_ADD);
    return 0;

err_destroy_device:
    device_destroy(ads1256_class, MKDEV(ads1256_major, 0));
err_destroy_class:
    class_destroy(ads1256_class);
err_del_cdev:
    cdev_del(&ads1256_cdev);
err_unreg_chrdev:
    unregister_chrdev(ads1256_major, ADS1256_NAME);
err_put_drdy:
    if (data->drdy_gpio)
        gpiod_put(data->drdy_gpio);
err_free_data:
    spi_set_drvdata(spi, NULL);
    devm_kfree(&spi->dev, data);
    return ret;
}

static int ads1256_remove(struct spi_device *spi) {
    struct ads1256_data *data = spi_get_drvdata(spi);

    if (data) {
        gpiod_put(data->drdy_gpio);  // Release DRDY GPIO
        device_destroy(ads1256_class, MKDEV(ads1256_major, 0));
        class_destroy(ads1256_class);
        cdev_del(&ads1256_cdev);  // Clean up cdev
        unregister_chrdev(ads1256_major, ADS1256_NAME);
    }
    return 0;  // Return an int to match the spi_driver.remove signature
}

static const struct spi_device_id ads1256_id[] = {
    { "ads1256", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, ads1256_id);

static struct spi_driver ads1256_driver = {
    .driver = {
        .name = ADS1256_NAME,
        .owner = THIS_MODULE,
    },
    .probe = ads1256_probe,
    .remove = ads1256_remove,
    .id_table = ads1256_id,
};

static int __init ads1256_init(void) {
    return spi_register_driver(&ads1256_driver);
}

static void __exit ads1256_exit(void) {
    spi_unregister_driver(&ads1256_driver);
}

module_init(ads1256_init);
module_exit(ads1256_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Basic ADS1256 ADC Driver Relying on SPI Controller CS");