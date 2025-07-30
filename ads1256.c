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

// ADS1256 Commands (see p34)
#define ADS1256_CMD_WAKEUP	    0x00  // Completes SYNC and Exits Standby Mode 0000  0000 (00h)
#define ADS1256_CMD_RDATA       0x01  // Read Data 0000  0001 (01h)
#define ADS1256_CMD_RDATAC	    0x03  // Read Data Continuously 0000   0011 (03h)
#define ADS1256_CMD_SDATAC	    0x0F  // Stop Read Data Continuously 0000   1111 (0Fh)
#define ADS1256_REG_secdCMD     0x00  // This is the second command byte that is to be used along with WREG and RREG
#define ADS1256_CMD_RREG        0x10  // Read from Register rrr 0001 rrrr (1xh)
#define ADS1256_CMD_WREG        0x50  // Write to Register rrr 0101 rrrr (5xh)
#define ADS1256_CMD_SELFCAL	    0xF0  // Offset and Gain Self-Calibration 1111    0000 (F0h)
#define ADS1256_CMD_SELFOCAL	0xF1  // Offset Self-Calibration 1111    0001 (F1h)
#define ADS1256_CMD_SELFGCAL	0xF2  // Gain Self-Calibration 1111    0010 (F2h)
#define ADS1256_CMD_SYOCAL	    0xF3  // System Offset Calibration 1111   0011 (F3h)
#define ADS1256_CMD_SYGCAL	    0xF4  // System Gain Calibration 1111    0100 (F4h)
#define ADS1256_CMD_SYNC	    0xFC  // Synchronize the A/D Conversion 1111   1100 (FCh)
#define ADS1256_CMD_STANDBY	    0xFD  // Begin Standby Mode 1111   1101 (FDh)
#define ADS1256_CMD_RESET       0xFE  // Reset to Power-Up Values 1111   1110 (FEh)
#define ADS1256_CMD_NOP	        0xFF

// ADS1256 Register Addresses (see p30)
#define ADS1256_REG_STATUS      0x00  // Status Register
#define ADS1256_REG_MUX         0x01  // MUX Register (channel selection)
#define ADS1256_REG_ADCON       0x02  // A/D Control Register (PGA, clock)
#define ADS1256_REG_DRATE       0x03  // Data Rate Register
#define ADS1256_REG_IO		    0x04  
#define ADS1256_REG_OFCAL0	    0x05  
#define ADS1256_REG_OFCAL1	    0x06  
#define ADS1256_REG_OFCAL2	    0x07  
#define ADS1256_REG_FSCAL0	    0x08  
#define ADS1256_REG_FSCAL1	    0x09  
#define ADS1256_REG_FSCAL2	    0x0A  

// Status Control Register (see p30)
#define ADS1256_STATUS_ID           0x30  // Factory Programmed Identification Bits (Read Only)
#define ADS1256_STATUS_RESET        0x01  // Reset STATUS Register
#define ADS1256_STATUS_ORDER_LSB    0x08  // Least significant Bit first, MSB is default 
#define ADS1256_STATUS_ACAL_ON      0x04  // Auto Calibration Enabled, Disabled by default
#define ADS1256_STATUS_BUFFER_ON    0x02  // BUffer Enabled, Disabled by default

// Multiplexer Control Register (see p31)
//      ADS125x common channels
#define ADS1256_MUX_AIN0		    0x00
#define ADS1256_MUX_AIN1	    	0x01
#define ADS1256_MUX_AINCOM	        0x0c
//      ADS1256 only channels
#define ADS1256_MUX_AIN2		    0x02
#define ADS1256_MUX_AIN3		    0x03
#define ADS1256_MUX_AIN4		    0x04
#define ADS1256_MUX_AIN5		    0x05
#define ADS1256_MUX_AIN6		    0x06
#define ADS1256_MUX_AIN7		    0x07
#define ADS1256_MAX_CHANNELS	8

// A/D Control Register (see p31)
//      D0/CLKOUT Clock Out Rate Setting
#define ADS1256_ADCON_CLK_1		    0x20  // Clock Out Frequency = fCLKIN
#define ADS1256_ADCON_CLK_2	    	0x40  // Clock Out Frequency = fCLKIN/2
#define ADS1256_ADCON_CLK_4		    0x60  // Clock Out Frequency = fCLKIN/4
//      Sensor Detection Current Sources
#define ADS1256_ADCON_SDCS_0d5		0x08  // Sensor Detect Current 0.5uA
#define ADS1256_ADCON_SDCS_2		0x10  // Sensor Detect Current 2uA
#define ADS1256_ADCON_SDCS_10		0x18  // Sensor Detect Current 10uA
//      Programmable Gain Amplifier Setting
#define ADS1256_ADCON_PGA_1		    0x00  // Programmable Gain Amplifier Setting, of 1
#define ADS1256_ADCON_PGA_2		    0x01  // Programmable Gain Amplifier Setting, of 2
#define ADS1256_ADCON_PGA_4		    0x02  // Programmable Gain Amplifier Setting, of 4
#define ADS1256_ADCON_PGA_8		    0x03  // Programmable Gain Amplifier Setting, of 8
#define ADS1256_ADCON_PGA_16		0x04  // Programmable Gain Amplifier Setting, of 16
#define ADS1256_ADCON_PGA_32		0x05  // Programmable Gain Amplifier Setting, of 32
#define ADS1256_ADCON_PGA_64		0x06  // Programmable Gain Amplifier Setting, of 64

// A/D Data Rate Register (see p32)
#define ADS1256_DRATE_SPS_30000		0xF0  // 30,000 SPS, Samples per sec
#define ADS1256_DRATE_SPS_15000		0xE0  // 15,000 SPS, Samples per sec
#define ADS1256_DRATE_SPS_7500		0xD0  // 7,500 SPS, Samples per sec
#define ADS1256_DRATE_SPS_3750		0xC0  // 3,750 SPS, Samples per sec
#define ADS1256_DRATE_SPS_2000		0xB0  // 2,000 SPS, Samples per sec
#define ADS1256_DRATE_SPS_1000		0xA1  // 1,000 SPS, Samples per sec
#define ADS1256_DRATE_SPS_500		0x92  // 500 SPS, Samples per sec
#define ADS1256_DRATE_SPS_100		0x82  // 100 SPS, Samples per sec
#define ADS1256_DRATE_SPS_60		0x72  // 60 SPS, Samples per sec
#define ADS1256_DRATE_SPS_50		0x63  // 50 SPS, Samples per sec
#define ADS1256_DRATE_SPS_30		0x53  // 30 SPS, Samples per sec
#define ADS1256_DRATE_SPS_25		0x43  // 25 SPS, Samples per sec
#define ADS1256_DRATE_SPS_15		0x33  // 15 SPS, Samples per sec
#define ADS1256_DRATE_SPS_10		0x20  // 10 SPS, Samples per sec
#define ADS1256_DRATE_SPS_5 		0x13  // 5 SPS, Samples per sec
#define ADS1256_DRATE_SPS_2d5		0x03  // 2.5 SPS, Samples per sec

// GPIO Control Register (see p32)
#define ADS1256_IO_RESET		0xE0  // Reset I/O Register to Default
#define ADS1256_IO_DIR3_IN		0x80  // DIR3 - Digital I/O Direction for Pin D3
#define ADS1256_IO_DIR2_IN		0x40  // DIR2 - Digital I/O Direction for Pin D2
#define ADS1256_IO_DIR1_IN		0x20  // DIR1 - Digital I/O Direction for Pin D1
#define ADS1256_IO_DIR0_IN		0x10  // DIR0 - Digital I/O Direction for Pin D0/CLKOUT

// Default Configuration Values
#define ADS1256_MUX_DEFAULT         0x01  // Select AIN0 (channel 0) as input
#define ADS1256_ADCON_DEFAULT       0x20  // PGA = 1 (gain 1), clock = internal
#define ADS1256_DRATE_DEFAULT       0x03  // Data rate = 30 SPS (samples per second)

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
    // pr_info("Matching dev=%p, devt=0x%u, target=0x%u\n", dev, (unsigned int)dev->devt, (unsigned int)target);
    pr_info("Matching dev=%p, devt=0x%llu, target=0x%llu\n", dev, (unsigned long long)dev->devt, (unsigned long long)target);
    return dev && dev->devt == target;
}

/* SPI Transfer Helper */
static int ads1256_spi_transfer(struct ads1256_data *data, u8 *tx_buf, u8 *rx_buf, int len)
{
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = len,
    };
    struct spi_message m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spi_sync(data->spi, &m);
}

/* Write to ADS1256 Register */
static int ads1256_write_reg(struct ads1256_data *data, u8 reg, u8 value)
{
    u8 tx_buf[3] = { ADS1256_CMD_WREG | reg, ADS1256_REG_secdCMD, value };
    return ads1256_spi_transfer(data, tx_buf, NULL, 3);
}

/* Read from ADS1256 Register */
static u8 __attribute__((unused)) ads1256_read_reg(struct ads1256_data *data, u8 reg)
{
    u8 tx_buf[2] = { ADS1256_CMD_RREG | reg, ADS1256_REG_secdCMD };
    u8 rx_buf[2] = { 0 };
    ads1256_spi_transfer(data, tx_buf, rx_buf, 2);
    return rx_buf[1];
}

// Reset the ADS1256
static int ads1256_reset(struct ads1256_data *data) {
    int ret = 0;
    u8 reset_cmd = ADS1256_CMD_RESET;
    u8 sdatac_cmd = ADS1256_CMD_SDATAC;

    mutex_lock(&data->lock);
    ret = ads1256_spi_transfer(data, &reset_cmd, NULL, 1);
    if (ret) {
        dev_err(&data->spi->dev, "ADS1256 reset failed: %d\n", ret);
        goto out;
    }
    usleep_range(500, 1000);

    ret = ads1256_spi_transfer(data, &sdatac_cmd, NULL, 1);
    if (ret) {
        dev_err(&data->spi->dev, "ADS1256 stop read data continuously failed: %d\n", ret);
        goto out;
    }
    usleep_range(500, 1000);

    dev_info(&data->spi->dev, "ADS1256 reset successful\n");
out:
    mutex_unlock(&data->lock);
    return ret;
}

// Configure the ADS1256
static int ads1256_configure(struct ads1256_data *data) {
    int ret;
    u8 selfcal_cmd = ADS1256_CMD_SELFCAL;

    mutex_lock(&data->lock);
    // Set STATUS Reg with buffer on.
    ret = ads1256_write_reg(data, ADS1256_REG_STATUS, ADS1256_STATUS_ID | ADS1256_STATUS_BUFFER_ON);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set Buffer ON: %d\n", ret);
        goto out;
    }

    // Set MUX (AIN0 as input, AINCOM as reference)
    ret = ads1256_write_reg(data, ADS1256_REG_MUX, ADS1256_MUX_DEFAULT);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set MUX: %d\n", ret);
        goto out;
    }

    // Set ADCON (PGA = 1, internal clock)
    ret = ads1256_write_reg(data, ADS1256_REG_ADCON, ADS1256_ADCON_DEFAULT);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set ADCON: %d\n", ret);
        goto out;
    }

    // Set DRATE (30 SPS)
    ret = ads1256_write_reg(data, ADS1256_REG_DRATE, ADS1256_DRATE_DEFAULT);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set DRATE: %d\n", ret);
        goto out;
    }

    // Start conversions
    ret = ads1256_spi_transfer(data, &selfcal_cmd, NULL, 1);
    if (ret == 0) {
        // Wait after reset (per datasheet, 0.5 ms minimum) using usleep_range for robustness
        usleep_range(500, 1000);  // Sleep 500-1000 us (0.5-1 ms)
        dev_info(&data->spi->dev, "Self calibration completed\n");
    } else {
        dev_err(&data->spi->dev, "Failed self calibration: %d\n", ret);
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
    int timeout = ADS1256_DRDY_TIMEOUT / 1000;  // Convert to iterations (33 iterations at 1 ms)
    u8 rdata_cmd = ADS1256_CMD_RDATA;

    mutex_lock(&data->lock);
    // Poll DRDY (active-low, so wait for it to go low)
    if (data->drdy_gpio) {
        while (gpiod_get_value_cansleep(data->drdy_gpio) && timeout--) {
            usleep_range(1000, 2000);
        }
        if (timeout <= 0) {
            dev_err(&data->spi->dev, "DRDY timeout waiting for data ready\n");
            ret = -ETIMEDOUT;
            goto out;
        }
    } else {
        usleep_range(ADS1256_DRDY_TIMEOUT, ADS1256_DRDY_TIMEOUT + 1000);  // Fixed delay
        dev_dbg(&data->spi->dev, "No DRDY GPIO, using fixed delay\n");
    }
    dev_dbg(&data->spi->dev, "DRDY asserted, proceeding to read\n");

    // Send RDATA command
    dev_dbg(&data->spi->dev, "Sending RDATA command, TX buffer: 0x%02x\n", data->tx_buf[0]);
    ret = ads1256_spi_transfer(data, &rdata_cmd, NULL, 1);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to send RDATA: %d\n", ret);
        goto out;
    }

    ret = ads1256_spi_transfer(data, NULL, data->rx_buf, 3);
    if (ret) {
        dev_err(&data->spi->dev, "SPI read failed: %d\n", ret);
        goto out;
    }

    *value = (data->rx_buf[0] << 16) | (data->rx_buf[1] << 8) | data->rx_buf[2];
    if (*value & 0x800000) {
        *value |= 0xFF000000;  // Sign-extend to 32 bits
    }
    dev_dbg(&data->spi->dev, "Read sample: %d (0x%08x, raw: 0x%02x 0x%02x 0x%02x)\n",
            (int32_t)*value, *value, data->rx_buf[0], data->rx_buf[1], data->rx_buf[2]);

out:
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

    dev_dbg(&data->spi->dev, "Successfully read sample 0x%08x to user space\n", sample);
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
    // dev = container_of_safe(inode->i_cdev->kobj.parent, struct device, kobj);
    dev = container_of(inode->i_cdev->kobj.parent, struct device, kobj);
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

static ssize_t last_sample_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    u32 sample;
    int ret = ads1256_read_sample(data, &sample);
    if (ret)
        return ret;
    return sprintf(buf, "0x%08x\n", sample);
}
static DEVICE_ATTR_RO(last_sample);

static int ads1256_probe(struct spi_device *spi) {
    struct ads1256_data *data;
    int ret;
    int attempt = 0;
    const int MAX_ATTEMPTS = 10;  // Increased attempts for robustness
	
	/* Allocate device data */
    data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        dev_err(&spi->dev, "Failed to allocate memory for driver data: %d\n", -ENOMEM);
        return -ENOMEM;
    }

    data->spi = spi;
    mutex_init(&data->lock);
    spi_set_drvdata(spi, data);

    // Get DRDY GPIO from device tree (e.g., named "drdy")
    data->drdy_gpio = devm_gpiod_get(&spi->dev, "drdy", GPIOD_IN);
    if (IS_ERR(data->drdy_gpio)) {
        dev_warn(&spi->dev, "DRDY GPIO not found, using delay-based polling\n");
        data->drdy_gpio = NULL;
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
        return ret;
    }
    dev_info(&spi->dev, "SPI setup succeeded with mode 1\n");

	/* Register Character Device */
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

    ret = device_create_file(ads1256_device, &dev_attr_last_sample);
    if (ret)
        dev_warn(&spi->dev, "Failed to create sysfs file: %d\n", ret);

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
    return ret;
}

// SPI remove function
static int ads1256_remove(struct spi_device *spi) {
    struct ads1256_data *data = spi_get_drvdata(spi);

    if (data) {
        gpiod_put(data->drdy_gpio);  // Release DRDY GPIO
        device_destroy(ads1256_class, MKDEV(ads1256_major, 0));
        class_destroy(ads1256_class);
        cdev_del(&ads1256_cdev);  // Clean up cdev
        unregister_chrdev(ads1256_major, ADS1256_NAME);
    }
    spi_set_drvdata(spi, NULL);
	dev_info(&spi->dev, "ADS1256 removed\n");
    return 0;  // Return an int to match the spi_driver.remove signature
}

static const struct spi_device_id ads1256_id[] = {
    { "ads1256", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, ads1256_id);

static const struct of_device_id ads1256_of_match[] = {
    { .compatible = "ti,ads1256" },
    { }
};
MODULE_DEVICE_TABLE(of, ads1256_of_match);

static struct spi_driver ads1256_driver = {
    .driver = {
        .name = ADS1256_NAME,
        .owner = THIS_MODULE,
        .of_match_table = ads1256_of_match,
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