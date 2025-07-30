#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>            // For usleep_range
#include <linux/device.h>           // For device APIs, class_find_device
#include <linux/gpio/consumer.h>    // For GPIO handling
#include <linux/of.h>               // For device tree parsing
#include <linux/cdev.h>             // For cdev operations
#include <linux/interrupt.h>        // For DRDY interupt handling
#include <linux/ioctl.h>
#include <linux/ktime.h>            // For timestamps
#include <linux/workqueue.h>
#include <linux/poll.h>

// Define constants
#define ADS1256_NAME "ads1256"
#define ADS1256_DRDY_TIMEOUT 33000  // microseconds (33 ms for 30 SPS)
#define MAX_ADS1256_DEVICES 5  // SPI0, SPI3, SPI4, SPI5, SPI6
#define SYNC_TIMEOUT_MS 5000  // 5 seconds timeout

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

// Reset Configuration Values
#define ADS1256_MUX_RESET         0x01
#define ADS1256_ADCON_RESET       0x20
#define ADS1256_DRATE_RESET       0xF0
#define ADS1256_STATUS_RESET      0x01
#define ADS1256_IO_RESET          0xE0  // AIN0 vs. AINCOM as startup default

#define ADS1256_IOC_MAGIC 'a'
#define ADS1256_SET_DATA_RATE _IOW(ADS1256_IOC_MAGIC, 1, int)
#define ADS1256_SET_PGA_GAIN  _IOW(ADS1256_IOC_MAGIC, 2, int)
#define ADS1256_SET_SENSOR_DETECT _IOW(ADS1256_IOC_MAGIC, 4, int)

static atomic_t ads1256_device_count = ATOMIC_INIT(0);
static struct ads1256_data *ads1256_devices[MAX_ADS1256_DEVICES];
static DEFINE_SPINLOCK(device_list_lock);

static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug logging (0=off, 1=on)");

static struct gpio_desc *shared_sync_gpio;  // Global sync GPIO descriptor
static DEFINE_MUTEX(sync_mutex);            // Mutex for sync operations
static struct delayed_work sync_work;

struct ads1256_mux_sample {
    u8 mux_config;
    u32 value;
    u64 timestamp;  // Nanoseconds since boot
} __attribute__((packed));

static bool is_valid_drate(int drate) {
    switch (drate) {
    case ADS1256_DRATE_SPS_30000:
    case ADS1256_DRATE_SPS_15000:
    case ADS1256_DRATE_SPS_7500:
    case ADS1256_DRATE_SPS_3750:
    case ADS1256_DRATE_SPS_2000:
    case ADS1256_DRATE_SPS_1000:
    case ADS1256_DRATE_SPS_500:
    case ADS1256_DRATE_SPS_100:
    case ADS1256_DRATE_SPS_60:
    case ADS1256_DRATE_SPS_50:
    case ADS1256_DRATE_SPS_30:
    case ADS1256_DRATE_SPS_25:
    case ADS1256_DRATE_SPS_15:
    case ADS1256_DRATE_SPS_10:
    case ADS1256_DRATE_SPS_5:
    case ADS1256_DRATE_SPS_2d5:
        return true;
    default:
        return false;
    }
}

static bool is_valid_pga(int pga) {
    return pga >= ADS1256_ADCON_PGA_1 && pga <= ADS1256_ADCON_PGA_64;
}

static bool is_valid_mux_config(u8 mux_config) {
    u8 pos = (mux_config >> 4) & 0x0F;  // Positive input (AIN0-AIN7, AINCOM)
    u8 neg = mux_config & 0x0F;         // Negative input (AIN0-AIN7, AINCOM)

    // Valid inputs are AIN0-AIN7 (0x0-0x7) or AINCOM (0xC)
    if ((pos > 0x7 && pos != 0xC) || (neg > 0x7 && neg != 0xC))
        return false;

    // Allow any valid pair (single-ended or differential)
    return true;
}

// Driver data structure
struct ads1256_data {
    struct spi_device *spi;
    struct mutex lock;
    struct gpio_desc *drdy_gpio;
    struct gpio_desc *sync_gpio;  // Shared sync GPIO
    struct gpio_desc *reset_gpio;  // New field for reset pin
    u8 tx_buf[4];
    u8 rx_buf[4];
    wait_queue_head_t wait;
    bool data_ready;
    int irq;
    bool sysfs_last_sample;
    bool sysfs_calibrate;
    struct ads1256_mux_sample *mux_samples;
    size_t num_samples;
    bool sampling_active;
    struct ads1256_mux_sample last_sample;
    struct cdev cdev;           // Per-device cdev
    struct device *device;      // Per-device device pointer
    int dev_num;                // SPI bus number (0, 3, 4, 5, 6)
    dev_t devt;                 // Device number (major + minor)
    int data_rate;  // Per-device data rate
    int pga_gain;   // Per-device PGA gain
};

static struct class *ads1256_class;  // Single class for all devices
static int ads1256_major;            // Single major number for all devices

// Match function for class_find_device to find device by dev_t
static int ads1256_devt_match(struct device *dev, const void *data) {
    dev_t target = *(dev_t *)data;
    pr_info("Matching dev=%p, devt=0x%llu, target=0x%llu\n", dev, (unsigned long long)dev->devt, (unsigned long long)target);
    return dev && dev->devt == target;
}

/* SPI Transfer Helper */
static int ads1256_spi_transfer(struct ads1256_data *data, u8 *tx_buf, u8 *rx_buf, int len) {
    struct spi_transfer t = {
        .tx_buf = tx_buf ? tx_buf : NULL,
        .rx_buf = rx_buf ? rx_buf : NULL,
        .len = len,
    };
    struct spi_message m;

    if (!tx_buf && !rx_buf) {
        dev_err(&data->spi->dev, "Both tx_buf and rx_buf are NULL\n");
        return -EINVAL;
    }

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spi_sync(data->spi, &m);
}

/* Write to ADS1256 Register */
static int ads1256_write_reg(struct ads1256_data *data, u8 reg, u8 value) {
    u8 tx_buf[3] = { ADS1256_CMD_WREG | reg, ADS1256_REG_secdCMD, value };
    return ads1256_spi_transfer(data, tx_buf, NULL, 3);
}

/* Read from ADS1256 Register */
static u8 __attribute__((unused)) ads1256_read_reg(struct ads1256_data *data, u8 reg) {
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

    if (data->reset_gpio) {
        // Hardware reset: assert low, then high (active-low assumed)
        gpiod_set_value_cansleep(data->reset_gpio, 0);
        usleep_range(10, 20);  // Minimum pulse width per datasheet
        gpiod_set_value_cansleep(data->reset_gpio, 1);
        usleep_range(500, 1000);  // Wait for device to stabilize
        dev_info(&data->spi->dev, "ADS1256 hardware reset via GPIO\n");
    } else {
        // Software reset
        ret = ads1256_spi_transfer(data, &reset_cmd, NULL, 1);
        if (ret) {
            dev_err(&data->spi->dev, "ADS1256 software reset failed: %d\n", ret);
            goto out;
        }
        usleep_range(500, 1000);
        dev_info(&data->spi->dev, "ADS1256 software reset successful\n");
    }

    // Always send SDATAC after reset to stop continuous mode
    ret = ads1256_spi_transfer(data, &sdatac_cmd, NULL, 1);
    if (ret) {
        dev_err(&data->spi->dev, "ADS1256 stop read data continuously failed: %d\n", ret);
        goto out;
    }
    usleep_range(500, 1000);

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
    ret = ads1256_write_reg(data, ADS1256_REG_MUX, ADS1256_MUX_RESET);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set MUX: %d\n", ret);
        goto out;
    }

    // Set ADCON
    ret = ads1256_write_reg(data, ADS1256_REG_ADCON, data->pga_gain);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set ADCON: %d\n", ret);
        goto out;
    }

    // Set DRATE
    ret = ads1256_write_reg(data, ADS1256_REG_DRATE, data->data_rate);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set DRATE: %d\n", ret);
        goto out;
    }

    // Start conversions
    ret = ads1256_spi_transfer(data, &selfcal_cmd, NULL, 1);
    if (ret) {
        dev_err(&data->spi->dev, "Failed self calibration: %d\n", ret);
        goto out;
    }

    usleep_range(500, 1000);
    dev_info(&data->spi->dev, "ADS1256 configured: DRATE=0x%02x, PGA=0x%02x\n", data->data_rate, data->pga_gain);
out:
    mutex_unlock(&data->lock);
    return ret;
}

static int ads1256_get_timeout_us(struct ads1256_data *data) {
    switch (data->data_rate) {
    case ADS1256_DRATE_SPS_30000: return 33;    // 33 us
    case ADS1256_DRATE_SPS_15000: return 67;    // 67 us
    case ADS1256_DRATE_SPS_7500:  return 133;   // 133 us
    case ADS1256_DRATE_SPS_3750:  return 267;   // 267 us
    case ADS1256_DRATE_SPS_2000:  return 500;   // 500 us
    case ADS1256_DRATE_SPS_1000:  return 1000;  // 1 ms
    case ADS1256_DRATE_SPS_500:   return 2000;  // 2 ms
    case ADS1256_DRATE_SPS_100:   return 10000; // 10 ms
    case ADS1256_DRATE_SPS_60:    return 16667; // 16.667 ms
    case ADS1256_DRATE_SPS_50:    return 20000; // 20 ms
    case ADS1256_DRATE_SPS_30:    return 33333; // 33.333 ms
    case ADS1256_DRATE_SPS_25:    return 40000; // 40 ms
    case ADS1256_DRATE_SPS_15:    return 66667; // 66.667 ms
    case ADS1256_DRATE_SPS_10:    return 100000;// 100 ms
    case ADS1256_DRATE_SPS_5:     return 200000;// 200 ms
    case ADS1256_DRATE_SPS_2d5:   return 400000;// 400 ms
    default: return ADS1256_DRDY_TIMEOUT;
    }
}

// Read one sample from channel 0, with DRDY polling
static int ads1256_read_sample(struct ads1256_data *data, struct ads1256_mux_sample *sample) {
    int ret;
    int timeout = ads1256_get_timeout_us(data) / 1000 + 1;  // Convert to ms iterations, add 1 for margin
    u8 rdata_cmd = ADS1256_CMD_RDATA;

    mutex_lock(&data->lock);
    // Set the MUX for this sample
    ret = ads1256_write_reg(data, ADS1256_REG_MUX, sample->mux_config);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to set MUX to 0x%02x: %d\n", sample->mux_config, ret);
        sample->timestamp = ktime_get_ns();  // Timestamp the failure
        goto out;
    }
    mutex_unlock(&data->lock);  // Release lock before polling
    
    // Poll DRDY (active-low, so wait for it to go low)
    if (data->irq >= 0) {
        data->data_ready = false;  // Reset before waiting
        if (debug)
            dev_dbg(&data->spi->dev, "Waiting for DRDY interrupt\n");
        ret = wait_event_interruptible_timeout(data->wait, data->data_ready,
                                               usecs_to_jiffies(ads1256_get_timeout_us(data)));
        if (ret <= 0) {
            dev_err(&data->spi->dev, "DRDY interrupt timeout or signal\n");
            sample->timestamp = ktime_get_ns();  // Timestamp the timeout
            ret = ret ? : -ETIMEDOUT;
            goto out;
        }
    } else if (data->drdy_gpio) {
        int poll_delay_us = ads1256_get_timeout_us(data) / 10;  // e.g., 1/10th of the sample period
        if (poll_delay_us < 1) poll_delay_us = 1;  // Minimum 1 µs
        if (debug)
            dev_dbg(&data->spi->dev, "Polling DRDY GPIO\n");
        while (gpiod_get_value_cansleep(data->drdy_gpio) && timeout--) {
            usleep_range(poll_delay_us, poll_delay_us + 1);
        }
        if (timeout <= 0) {
            dev_err(&data->spi->dev, "DRDY timeout waiting for data ready\n");
            sample->timestamp = ktime_get_ns();  // Timestamp the timeout
            ret = -ETIMEDOUT;
            goto out;
        }
    } else {
        dev_err(&data->spi->dev, "No DRDY GPIO or IRQ available, cannot guarantee data readiness\n");
        sample->timestamp = ktime_get_ns();
        return -EINVAL;
    }
    if (debug)
        dev_dbg(&data->spi->dev, "DRDY asserted, proceeding to read\n");
    
    mutex_lock(&data->lock);  // Re-acquire lock for SPI transfer

    // Send RDATA command
    if (debug)
        dev_dbg(&data->spi->dev, "Sending RDATA command: 0x%02x\n", rdata_cmd);
    ret = ads1256_spi_transfer(data, &rdata_cmd, NULL, 1);
    if (ret) {
        dev_err(&data->spi->dev, "Failed to send RDATA: %d\n", ret);
        sample->timestamp = ktime_get_ns();  // Timestamp the failure
        goto out;
    }

    ret = ads1256_spi_transfer(data, NULL, data->rx_buf, 3);
    if (ret) {
        dev_err(&data->spi->dev, "SPI read failed: %d\n", ret);
        sample->timestamp = ktime_get_ns();  // Timestamp the failure
        goto out;
    }

    sample->value = (data->rx_buf[0] << 16) | (data->rx_buf[1] << 8) | data->rx_buf[2];
    if (sample->value & 0x800000) {
        sample->value |= 0xFF000000;  // Sign-extend to 32 bits
    }
    sample->timestamp = ktime_get_ns();  // Capture timestamp in nanoseconds
    data->last_sample = *sample;  // Store the entire sample
    if (debug)
        dev_dbg(&data->spi->dev, "Read sample for MUX 0x%02x: %d (0x%08x) at %llu ns\n",
                sample->mux_config, (int32_t)sample->value, sample->value, sample->timestamp);

out:
    mutex_unlock(&data->lock);
    return ret;
}

static ssize_t ads1256_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
    struct ads1256_data *data = filp->private_data;
    size_t num_samples = count / sizeof(struct ads1256_mux_sample);
    struct ads1256_mux_sample *new_samples;
    size_t i;

    if (count % sizeof(struct ads1256_mux_sample) || num_samples == 0) {
        dev_err(&data->spi->dev, "Invalid write count: %zu, must be multiple of %zu\n",
                count, sizeof(struct ads1256_mux_sample));
        return -EINVAL;
    }

    new_samples = kmalloc(count, GFP_KERNEL);
    if (!new_samples) {
        dev_err(&data->spi->dev, "Failed to allocate buffer for %zu samples\n", num_samples);
        return -ENOMEM;
    }

    if (copy_from_user(new_samples, buf, count)) {
        kfree(new_samples);
        dev_err(&data->spi->dev, "Failed to copy MUX configs from user space\n");
        return -EFAULT;
    }

    // Validate MUX configs
    for (i = 0; i < num_samples; i++) {
        if (!is_valid_mux_config(new_samples[i].mux_config)) {
            dev_err(&data->spi->dev, "Invalid MUX config 0x%02x at index %zu\n",
                    new_samples[i].mux_config, i);
            kfree(new_samples);
            return -EINVAL;
        }
        // Optionally log ignored fields if keeping this behavior
        if (debug && (new_samples[i].value || new_samples[i].timestamp)) {
            dev_dbg(&data->spi->dev, "Ignoring user-provided value 0x%08x and timestamp %llu for MUX 0x%02x\n",
                    new_samples[i].value, new_samples[i].timestamp, new_samples[i].mux_config);
        }
    }

    mutex_lock(&data->lock);
    kfree(data->mux_samples);
    data->mux_samples = new_samples;
    data->num_samples = num_samples;
    mutex_unlock(&data->lock);

    dev_dbg(&data->spi->dev, "Stored %zu MUX configs\n", num_samples);
    return count;
}

static ssize_t ads1256_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
    struct ads1256_data *data = filp->private_data;
    size_t num_samples = count / sizeof(struct ads1256_mux_sample);
    struct ads1256_mux_sample *samples_buf;
    int ret;
    size_t i;

    if (count % sizeof(struct ads1256_mux_sample) || num_samples == 0) {
        dev_err(&data->spi->dev, "Invalid read count: %zu, must be multiple of %zu\n",
                count, sizeof(struct ads1256_mux_sample));
        return -EINVAL;
    }

    mutex_lock(&data->lock);
    if (!data->mux_samples || data->num_samples == 0) {
        mutex_unlock(&data->lock);
        dev_err(&data->spi->dev, "No MUX configurations set, use write first\n");
        return -EINVAL;
    }
    if (num_samples > data->num_samples) {
        num_samples = data->num_samples;  // Limit to stored configs
        count = num_samples * sizeof(struct ads1256_mux_sample);
    }
    data->sampling_active = true;  // Set flag

    samples_buf = kmalloc(count, GFP_KERNEL);
    if (!samples_buf) {
        dev_err(&data->spi->dev, "Failed to allocate buffer for %zu samples\n", num_samples);
        data->sampling_active = false;
        mutex_unlock(&data->lock);
        return -ENOMEM;
    }

    // Copy all mux_configs at once
    for (i = 0; i < num_samples; i++) {
        samples_buf[i].mux_config = data->mux_samples[i].mux_config;
    }
    mutex_unlock(&data->lock);

    for (i = 0; i < num_samples; i++) {
        ret = ads1256_read_sample(data, &samples_buf[i]);
        if (ret) {
            if (i > 0 && !copy_to_user(buf, samples_buf, i * sizeof(struct ads1256_mux_sample))) {
                kfree(samples_buf);
                mutex_lock(&data->lock);
                data->sampling_active = false;
                mutex_unlock(&data->lock);
                if (debug)
                    dev_dbg(&data->spi->dev, "Partial read: %zu of %zu samples completed\n", i, num_samples);
                return i * sizeof(struct ads1256_mux_sample);
            }
            kfree(samples_buf);
            mutex_lock(&data->lock);
            data->sampling_active = false;
            mutex_unlock(&data->lock);
            return ret;
        }
    }

    if (copy_to_user(buf, samples_buf, count)) {
        kfree(samples_buf);
        mutex_lock(&data->lock);
        data->sampling_active = false;
        mutex_unlock(&data->lock);
        dev_err(&data->spi->dev, "Failed to copy %zu samples to user space\n", num_samples);
        return -EFAULT;
    }

    kfree(samples_buf);
    mutex_lock(&data->lock);
    data->sampling_active = false;  // Clear flag
    mutex_unlock(&data->lock);
    dev_dbg(&data->spi->dev, "Successfully read %zu samples to user space\n", num_samples);
    return count;
}

static int ads1256_open(struct inode *inode, struct file *filp) {
    struct device *dev;
    struct ads1256_data *data;
    dev_t devt;

    spin_lock(&device_list_lock);  // Protect device lookup
    devt = inode->i_rdev;
    dev = class_find_device(ads1256_class, NULL, &devt, ads1256_devt_match);
    if (!dev) {
        spin_unlock(&device_list_lock);
        pr_err("Failed to find device for devt=0x%u\n", (unsigned int)devt);
        return -ENODEV;
    }

    data = dev_get_drvdata(dev);
    if (!data || !data->spi) {
        put_device(dev);  // Balance refcount
        spin_unlock(&device_list_lock);
        pr_err("Invalid driver data or SPI device\n");
        return -ENODEV;
    }

    filp->private_data = data;
    put_device(dev);  // Balance refcount
    spin_unlock(&device_list_lock);

    pr_info("ads1256_open: Successfully opened with data=%p\n", data);
    return 0;
}

static long ads1256_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct ads1256_data *data = filp->private_data;
    int val;
    int ret;

    mutex_lock(&data->lock);
    if (data->sampling_active) {
        mutex_unlock(&data->lock);
        dev_err(&data->spi->dev, "Cannot change settings during active sampling\n");
        return -EBUSY;
    }

    switch (cmd) {
    case ADS1256_SET_DATA_RATE:
        if (copy_from_user(&val, (int __user *)arg, sizeof(val))) {
            mutex_unlock(&data->lock);
            return -EFAULT;
        }
        if (!is_valid_drate(val)) {
            dev_err(&data->spi->dev, "Invalid data rate: 0x%x\n", val);
            mutex_unlock(&data->lock);
            return -EINVAL;
        }
        ret = ads1256_write_reg(data, ADS1256_REG_DRATE, val);
        if (ret) {
            mutex_unlock(&data->lock);
            return ret;
        }
        data->data_rate = val;  // Update global for timeout calculation
        dev_info(&data->spi->dev, "Data rate set to 0x%x\n", val);
        break;

    case ADS1256_SET_PGA_GAIN:
        if (copy_from_user(&val, (int __user *)arg, sizeof(val))) {
            mutex_unlock(&data->lock);
            return -EFAULT;
        }
        if (!is_valid_pga(val)) {
            dev_err(&data->spi->dev, "Invalid PGA gain: 0x%x\n", val);
            mutex_unlock(&data->lock);
            return -EINVAL;
        }
        ret = ads1256_write_reg(data, ADS1256_REG_ADCON, val);
        if (ret) {
            mutex_unlock(&data->lock);
            return ret;
        }
        data->pga_gain = val;
        dev_info(&data->spi->dev, "PGA gain set to 0x%x\n", val);
        break;
        
    case ADS1256_SET_SENSOR_DETECT:
        int sdcs;
        if (copy_from_user(&sdcs, (int __user *)arg, sizeof(sdcs))) {
            mutex_unlock(&data->lock);
            return -EFAULT;
        }
        if (sdcs != ADS1256_ADCON_SDCS_0d5 && sdcs != ADS1256_ADCON_SDCS_2 && sdcs != ADS1256_ADCON_SDCS_10) {
            mutex_unlock(&data->lock);
            return -EINVAL;
        }
        ret = ads1256_write_reg(data, ADS1256_REG_ADCON, data->pga_gain | sdcs);
        if (ret) {
            mutex_unlock(&data->lock);
            return ret;
        }
        dev_info(&data->spi->dev, "Sensor detect set to 0x%x\n", sdcs);
        break;

    default:
        mutex_unlock(&data->lock);
        return -ENOTTY;
    }
    mutex_unlock(&data->lock);
    return 0;
}

static int ads1256_release(struct inode *inode, struct file *filp) {
    struct ads1256_data *data = filp->private_data;
    pr_info("ads1256_release: Device closed for data=%p\n", data);
    return 0;
}

static unsigned int ads1256_poll(struct file *filp, struct poll_table_struct *wait) {
    struct ads1256_data *data = filp->private_data;
    unsigned int mask = 0;

    mutex_lock(&data->lock);
    poll_wait(filp, &data->wait, wait);
    if (data->data_ready)
        mask |= POLLIN | POLLRDNORM;
    mutex_unlock(&data->lock);
    return mask;
}

static const struct file_operations ads1256_fops = {
    .owner = THIS_MODULE,
    .read = ads1256_read,
    .write = ads1256_write,
    .open = ads1256_open,
    .unlocked_ioctl = ads1256_ioctl,
    .release = ads1256_release,
    .poll = ads1256_poll,
};

static irqreturn_t ads1256_drdy_handler(int irq, void *dev_id) {
    struct ads1256_data *data = dev_id;
    data->data_ready = true;
    wake_up_interruptible(&data->wait);
    return IRQ_HANDLED;
}

static ssize_t current_sample_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    ssize_t ret;

    mutex_lock(&data->lock);
    ret = snprintf(buf, PAGE_SIZE, "0x%08x at %llu ns\n", data->last_sample.value, data->last_sample.timestamp);
    mutex_unlock(&data->lock);
    return ret;
}
static DEVICE_ATTR_RO(current_sample);

static ssize_t last_sample_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    struct ads1256_mux_sample sample = { .mux_config = ADS1256_MUX_RESET };
    int ret;

    ret = ads1256_read_sample(data, &sample);
    if (ret)
        return ret;

    return snprintf(buf, PAGE_SIZE, "0x%08x at %llu ns\n", sample.value, sample.timestamp);
}
static DEVICE_ATTR_RO(last_sample);

static ssize_t calibrate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    u8 selfcal_cmd = ADS1256_CMD_SELFCAL;
    int ret;

    mutex_lock(&data->lock);
    ret = ads1256_spi_transfer(data, &selfcal_cmd, NULL, 1);
    if (ret) {
        dev_err(&data->spi->dev, "Calibration failed: %d\n", ret);
    } else {
        usleep_range(500, 1000);
        dev_info(&data->spi->dev, "Calibration triggered\n");
    }
    mutex_unlock(&data->lock);
    return ret ? ret : count;
}
static DEVICE_ATTR_WO(calibrate);

static int ads1256_sync_all(void) {
    int ret = 0, i;
    u8 sync_cmd = ADS1256_CMD_SYNC;

    mutex_lock(&sync_mutex);
    if (!shared_sync_gpio) {
        pr_err("Sync GPIO not configured, cannot synchronize devices\n");
        mutex_unlock(&sync_mutex);
        return -EINVAL;
    }

    gpiod_set_value_cansleep(shared_sync_gpio, 0);
    usleep_range(10, 20);
    gpiod_set_value_cansleep(shared_sync_gpio, 1);
    usleep_range(10, 20);

    spin_lock(&device_list_lock);
    for (i = 0; i < MAX_ADS1256_DEVICES; i++) {
        struct ads1256_data *data = ads1256_devices[i];
        if (data) {
            mutex_lock(&data->lock);
            ret = ads1256_spi_transfer(data, &sync_cmd, NULL, 1);
            if (ret) {
                dev_err(&data->spi->dev, "Failed to send SYNC command on SPI%d: %d\n", data->dev_num, ret);
            } else {
                dev_info(&data->spi->dev, "Synchronization triggered on SPI%d\n", data->dev_num);
            }
            mutex_unlock(&data->lock);
            if (ret) break;
        }
    }
    spin_unlock(&device_list_lock);

    mutex_unlock(&sync_mutex);
    return ret;
}

static void ads1256_sync_work_handler(struct work_struct *work) {
    int ret = ads1256_sync_all();
    if (ret) {
        pr_err("Timeout sync failed: %d\n", ret);
    } else {
        pr_info("Timeout sync completed for %d devices\n", atomic_read(&ads1256_device_count));
    }
}

static ssize_t sync_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    int ret = ads1256_sync_all();
    if (ret) {
        return ret;
    }
    return count;
}
static DEVICE_ATTR_WO(sync);

static ssize_t data_rate_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    ssize_t ret;
    mutex_lock(&data->lock);
    ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", data->data_rate);
    mutex_unlock(&data->lock);
    return ret;
}

static ssize_t data_rate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    unsigned int val;
    int ret;

    if (kstrtouint(buf, 16, &val) < 0)
        return -EINVAL;

    mutex_lock(&data->lock);
    if (data->sampling_active) {
        mutex_unlock(&data->lock);
        dev_err(&data->spi->dev, "Cannot change data rate during active sampling\n");
        return -EBUSY;
    }
    if (!is_valid_drate(val)) {
        mutex_unlock(&data->lock);
        dev_err(&data->spi->dev, "Invalid data rate: 0x%x\n", val);
        return -EINVAL;
    }
    ret = ads1256_write_reg(data, ADS1256_REG_DRATE, val);
    if (ret) {
        mutex_unlock(&data->lock);
        return ret;
    }
    data->data_rate = val;
    mutex_unlock(&data->lock);
    return count;
}
static DEVICE_ATTR_RW(data_rate);

static ssize_t pga_gain_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    ssize_t ret;
    mutex_lock(&data->lock);
    ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", data->pga_gain);
    mutex_unlock(&data->lock);
    return ret;
}

static ssize_t pga_gain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct ads1256_data *data = dev_get_drvdata(dev);
    unsigned int val;
    int ret;

    if (kstrtouint(buf, 16, &val) < 0)
        return -EINVAL;

    mutex_lock(&data->lock);
    if (data->sampling_active) {
        mutex_unlock(&data->lock);
        dev_err(&data->spi->dev, "Cannot change PGA gain during active sampling\n");
        return -EBUSY;
    }
    if (!is_valid_pga(val)) {
        mutex_unlock(&data->lock);
        dev_err(&data->spi->dev, "Invalid PGA gain: 0x%x\n", val);
        return -EINVAL;
    }
    ret = ads1256_write_reg(data, ADS1256_REG_ADCON, val);
    if (ret) {
        mutex_unlock(&data->lock);
        return ret;
    }
    data->pga_gain = val;
    mutex_unlock(&data->lock);
    return count;
}
static DEVICE_ATTR_RW(pga_gain);

static int ads1256_probe(struct spi_device *spi) {
    struct ads1256_data *data;
    int ret;
    int bus_num = spi->master->bus_num;
    char dev_name[16];
    int device_index;

    // Only support SPI0, SPI3, SPI4, SPI5, SPI6
    if (bus_num != 0 && bus_num != 3 && bus_num != 4 && bus_num != 5 && bus_num != 6) {
        dev_err(&spi->dev, "Unsupported SPI bus %d (only 0, 3, 4, 5, 6 allowed)\n", bus_num);
        return -EINVAL;
    }
	
	/* Allocate device data */
    data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        dev_err(&spi->dev, "Failed to allocate memory for driver data: %d\n", -ENOMEM);
        return -ENOMEM;
    }

    data->spi = spi;
    data->dev_num = bus_num;
    mutex_init(&data->lock);
    data->mux_samples = NULL;
    data->num_samples = 0;
    data->last_sample.mux_config = ADS1256_MUX_RESET;   // Default MUX
    data->last_sample.value = 0;                        // Default value
    data->last_sample.timestamp = 0;                    // Default timestamp
    data->data_rate = ADS1256_DRATE_SPS_30000;          // Default value
    data->pga_gain = ADS1256_ADCON_PGA_1;               // Default value
    data->sampling_active = false;  // Add this line after other initializations
    spi_set_drvdata(spi, data);

    // DRDY GPIO setup
    // Get DRDY GPIO from device tree (e.g., named "drdy")
    data->drdy_gpio = devm_gpiod_get(&spi->dev, "drdy", GPIOD_IN);
    if (IS_ERR(data->drdy_gpio)) {
        dev_warn(&spi->dev, "DRDY GPIO not found, using delay-based polling\n");
        data->drdy_gpio = NULL;
    }
    dev_info(&spi->dev, "DRDY GPIO retrieved: %p\n", data->drdy_gpio);
    data->irq = -1;  // Default to no IRQ
    init_waitqueue_head(&data->wait);  // Initialize unconditionally
    if (!IS_ERR(data->drdy_gpio)) {
        data->irq = gpiod_to_irq(data->drdy_gpio);
        if (data->irq >= 0) {
            ret = request_irq(data->irq, ads1256_drdy_handler, IRQF_TRIGGER_FALLING, "ads1256_drdy", data);
            if (ret) {
                dev_warn(&spi->dev, "Failed to request IRQ: %d, falling back to polling\n", ret);
                data->irq = -1;
            } else {
                dev_info(&spi->dev, "Using DRDY IRQ: %d\n", data->irq);
            }
        }
    }

    // Sync GPIO setup (only request once)
    mutex_lock(&sync_mutex);
    if (!shared_sync_gpio) {
        shared_sync_gpio = devm_gpiod_get(&spi->dev, "sync", GPIOD_OUT_HIGH);
        if (IS_ERR(shared_sync_gpio)) {
            ret = PTR_ERR(shared_sync_gpio);
            dev_warn(&spi->dev, "Sync GPIO not found, synchronization disabled: %d\n", ret);
            shared_sync_gpio = NULL;
        } else {
            dev_info(&spi->dev, "Shared sync GPIO retrieved: %p\n", shared_sync_gpio);
            INIT_DELAYED_WORK(&sync_work, ads1256_sync_work_handler);  // Initialize workqueue
        }
    }
    data->sync_gpio = shared_sync_gpio;  // Assign shared GPIO to this instance
    mutex_unlock(&sync_mutex);

    // Reset GPIO setup (optional)
    data->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(data->reset_gpio)) {
        ret = PTR_ERR(data->reset_gpio);
        dev_warn(&spi->dev, "Failed to get reset GPIO: %d, falling back to software reset\n", ret);
        data->reset_gpio = NULL;
    } else if (data->reset_gpio) {
        dev_info(&spi->dev, "Reset GPIO retrieved: %p\n", data->reset_gpio);
    }

    // SPI setup
    // Force SPI mode 1 (CPOL=0, CPHA=1) as per ADS1256 datasheet
    spi->mode = SPI_MODE_1;
    spi->bits_per_word = 8;
    if (spi->max_speed_hz == 0 || spi->max_speed_hz > 10000000) {  // ADS1256 max is 10 MHz
        dev_warn(&spi->dev, "Invalid spi-max-frequency %u Hz, using default 2500000 Hz\n", spi->max_speed_hz);
        spi->max_speed_hz = 2500000;  // Fallback if not set or out of range
    }

    dev_info(&spi->dev, "Attempting SPI setup with mode %d, bits_per_word %d, max_speed_hz %d\n",
             spi->mode, spi->bits_per_word, spi->max_speed_hz);

    ret = spi_setup(spi);
    if (ret) {
        dev_err(&spi->dev, "SPI setup with mode 1 failed: %d\n", ret);
        return ret;
    }
    dev_info(&spi->dev, "SPI setup succeeded with mode 1\n");

	/* Register Character Device */
    // Register character device (one per SPI device)
    if (!ads1256_major) {
        ret = register_chrdev(0, ADS1256_NAME, &ads1256_fops);
        if (ret < 0) {
            dev_err(&spi->dev, "Failed to register chrdev: %d\n", ret);
            return ret;
        }
        ads1256_major = ret;
        dev_info(&spi->dev, "Registered chrdev with major number %d\n", ads1256_major);

        ads1256_class = class_create(THIS_MODULE, ADS1256_NAME);
        if (IS_ERR(ads1256_class)) {
            ret = PTR_ERR(ads1256_class);
            unregister_chrdev(ads1256_major, ADS1256_NAME);
            dev_err(&spi->dev, "Failed to create class: %d\n", ret);
            return ret;
        }
        dev_info(&spi->dev, "Class created: %p\n", ads1256_class);
    }

    data->devt = MKDEV(ads1256_major, bus_num);
    cdev_init(&data->cdev, &ads1256_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->devt, 1);
    if (ret) {
        dev_err(&spi->dev, "Failed to add cdev for SPI%d: %d\n", bus_num, ret);
        goto err_cdev;
    }
    dev_info(&spi->dev, "cdev added: major=%d, minor=0, ret=%d\n", ads1256_major, ret);

    // Create device node (e.g., /dev/ads1256_0, /dev/ads1256_3)
    snprintf(dev_name, sizeof(dev_name), "ads1256_%d", bus_num);
    data->device = device_create(ads1256_class, &spi->dev, data->devt, data, dev_name);
    if (IS_ERR(data->device)) {
        ret = PTR_ERR(data->device);
        dev_err(&spi->dev, "Failed to create device %s: %d\n", dev_name, ret);
        goto err_device;
    }

    // Log device creation
    dev_info(&spi->dev, "Device created: %s, major=%d, minor=%d\n", dev_name, MAJOR(data->devt), MINOR(data->devt));

    // Sysfs attributes
    ret = device_create_file(data->device, &dev_attr_current_sample);
    if (ret)
        dev_warn(&spi->dev, "Failed to create current_sample sysfs: %d\n", ret);

    ret = device_create_file(data->device, &dev_attr_last_sample);
    if (ret) {
        dev_warn(&spi->dev, "Failed to create last_sample sysfs: %d\n", ret);
    } else {
        data->sysfs_last_sample = true;
    }

    ret = device_create_file(data->device, &dev_attr_calibrate);
    if (ret) {
        dev_warn(&spi->dev, "Failed to create calibrate sysfs: %d\n", ret);
    } else {
        data->sysfs_calibrate = true;
    }

    ret = device_create_file(data->device, &dev_attr_sync);
    if (ret) {
        dev_warn(&spi->dev, "Failed to create sync sysfs: %d\n", ret);
    }

    ret = device_create_file(data->device, &dev_attr_data_rate);
    if (ret)
        dev_warn(&data->spi->dev, "Failed to create data_rate sysfs: %d\n", ret);

    ret = device_create_file(data->device, &dev_attr_pga_gain);
    if (ret)
        dev_warn(&data->spi->dev, "Failed to create pga_gain sysfs: %d\n", ret);

    // Initialize and configure the ADS1256
    ret = ads1256_reset(data);
    if (ret) {
        dev_err(&spi->dev, "Failed to reset ADS1256: %d\n", ret);
        goto err_init;
    }
    ret = ads1256_configure(data);
    if (ret) {
        dev_err(&spi->dev, "Failed to configure ADS1256: %d\n", ret);
        goto err_init;
    }

    // Register this device in the global list
    spin_lock(&device_list_lock);
    switch (bus_num) {
    case 0: device_index = 0; break;
    case 3: device_index = 1; break;
    case 4: device_index = 2; break;
    case 5: device_index = 3; break;
    case 6: device_index = 4; break;
    default: device_index = -1; break;  // Should never happen due to earlier check
    }
    if (device_index >= 0 && device_index < MAX_ADS1256_DEVICES) {
        ads1256_devices[device_index] = data;
    }
    spin_unlock(&device_list_lock);

    // Sync all devices now that a new one is added
    ret = ads1256_sync_all();
    if (ret) {
        dev_err(&spi->dev, "Failed to sync devices after adding SPI%d: %d\n", bus_num, ret);
    } else {
        dev_info(&spi->dev, "Synced %d devices after adding SPI%d\n", atomic_read(&ads1256_device_count), bus_num);
    }

    // Schedule timeout to re-sync if no new devices are added
    if (shared_sync_gpio) {
        mod_delayed_work(system_wq, &sync_work, msecs_to_jiffies(SYNC_TIMEOUT_MS));
    }

    dev_info(&spi->dev, "ADS1256 on SPI%d initialized, device %s created\n", bus_num, dev_name);
    kobject_uevent(&data->device->kobj, KOBJ_ADD);
    atomic_inc(&ads1256_device_count);  // Add this line to track active devices
    return 0;

err_init:
    device_remove_file(data->device, &dev_attr_sync);
    if (data->sysfs_calibrate)
        device_remove_file(data->device, &dev_attr_calibrate);
    if (data->sysfs_last_sample)
        device_remove_file(data->device, &dev_attr_last_sample);
    device_remove_file(data->device, &dev_attr_current_sample);
    device_destroy(ads1256_class, data->devt);
err_device:
    cdev_del(&data->cdev);
err_cdev:
    if (bus_num == 0) {  // Only cleanup class/major on first failure
        class_destroy(ads1256_class);
        unregister_chrdev(ads1256_major, ADS1256_NAME);
        ads1256_class = NULL;
        ads1256_major = 0;
    }
    return ret;
}

// SPI remove function
static int ads1256_remove(struct spi_device *spi) {
    struct ads1256_data *data = spi_get_drvdata(spi);
    int device_index;

    if (data) {
        if (data->irq >= 0) {
            free_irq(data->irq, data);
            data->irq = -1;  // Prevent double-free attempts
        }

        mutex_lock(&data->lock);
        kfree(data->mux_samples);
        data->mux_samples = NULL;
        data->num_samples = 0;
        mutex_unlock(&data->lock);

        // Only remove sysfs files and device if they were created
        if (data->device) {
            device_remove_file(data->device, &dev_attr_sync);
            if (data->sysfs_last_sample)
                device_remove_file(data->device, &dev_attr_last_sample);
            if (data->sysfs_calibrate)
                device_remove_file(data->device, &dev_attr_calibrate);
            device_remove_file(data->device, &dev_attr_current_sample);  // Always attempt, safe if not present
            device_remove_file(data->device, &dev_attr_data_rate);
            device_remove_file(data->device, &dev_attr_pga_gain);
            device_destroy(ads1256_class, data->devt);  // Use per-device devt
            data->device = NULL;  // Prevent double-destroy
        }

        cdev_del(&data->cdev);

        // Remove from device list
        spin_lock(&device_list_lock);
        switch (data->dev_num) {
        case 0: device_index = 0; break;
        case 3: device_index = 1; break;
        case 4: device_index = 2; break;
        case 5: device_index = 3; break;
        case 6: device_index = 4; break;
        default: device_index = -1; break;
        }
        if (device_index >= 0 && device_index < MAX_ADS1256_DEVICES) {
            ads1256_devices[device_index] = NULL;
        }
        atomic_dec(&ads1256_device_count);
        if (atomic_read(&ads1256_device_count) == 0 && shared_sync_gpio) {
            cancel_delayed_work_sync(&sync_work);  // Cancel timeout if no devices remain
        }
        spin_unlock(&device_list_lock);

        dev_info(&spi->dev, "ADS1256 on SPI%d removed\n", data->dev_num);
    }
    spi_set_drvdata(spi, NULL);
    return 0;
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
    if (ads1256_class) {
        class_destroy(ads1256_class);
        unregister_chrdev(ads1256_major, ADS1256_NAME);
    }
}

module_init(ads1256_init);
module_exit(ads1256_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Basic ADS1256 ADC Driver Relying on SPI Controller CS");