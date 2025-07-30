# ADS1256 Linux Kernel Driver
This is a Linux kernel driver for the Texas Instruments ADS1256, a high-precision, 24-bit analog-to-digital converter (ADC) with eight single-ended or four differential input channels. Designed for the Raspberry Pi, this driver supports multiple ADS1256 devices connected to SPI buses (SPI0, SPI3, SPI4, SPI5, and SPI6). It offers advanced features such as configurable data rates, programmable gain amplifier (PGA) settings, sensor detection, sample buffering, and synchronization across multiple devices, making it suitable for applications requiring precise and simultaneous analog-to-digital conversion.

## Features
- **Multiple SPI Bus Support:** Compatible with SPI0, SPI3, SPI4, SPI5, and SPI6 on Raspberry Pi.
- **Configurable Data Rates:** Supports sampling rates from 2.5 SPS to 30,000 SPS.
- **Programmable Gain Amplifier (PGA):** Adjustable gains of 1, 2, 4, 8, 16, 32, and 64.
- **Sensor Detection:** Configurable current sources (0.5 µA, 2 µA, 10 µA) for sensor validation.
- **Sample Buffering:** Circular buffer for continuous sampling, with configurable size (default 1024 samples).
- **Device Synchronization:** Shared sync GPIO ensures simultaneous sampling across multiple ADS1256 devices.
- **Sysfs Interface:** Attributes for monitoring samples, triggering calibration, and configuring settings.
- **Device Tree Support:** Customizable GPIO pin assignments via overlay.

## Installation
### Prerequisites
- **Hardware:** Raspberry Pi (tested on Raspberry Pi 4 with Raspberry Pi OS).
- **Software:**
  - Linux kernel headers (`sudo apt install raspberrypi-kernel-headers`).
  - SPI enabled in `/boot/config.txt` (add `dtparam=spi=on` if not present).
- **Permissions:** Root or `sudo` access required for building and loading the driver.

### Building the Driver
- Clone the repository:
  ```
  git clone https://github.com/yourusername/ads1256-driver.git
  cd ads1256-driver
  ```
- Compile the kernel module:
  ```
  make
  ```
### Loading the Module
- Remove any existing ADS1256 overlay and module:
```
sudo dtoverlay -r ads1256
sudo rmmod ads1256
```
- Insert the compiled module:
```
sudo insmod ads1256.ko
```
### Applying the Device Tree Overlay
- Compile the device tree overlay:
```
dtc -@ -I dts -O dtb -o ads1256.dtbo ads1256-overlay.dts
```
- Load the overlay:
```
sudo dtoverlay ./ads1256.dtbo
```
- Verify the overlay is applied:
```
ls /sys/kernel/config/device-tree/overlays/
```
Look for `ads1256` in the output.

### Automation with `build.sh`
The provided `build.sh` script automates the above steps. Run it with:
```
sudo ./build.sh
```
**Note:** Ensure the script is executable (chmod +x build.sh) and run as root.

## Usage
Upon loading, the driver creates character devices under `/dev/ads1256_X.Y`, where `X` is the SPI bus number (0, 3, 4, 5, or 6) and `Y` is the chip select (typically 0).

### Device Files
- `/dev/ads1256_0.0`: SPI0, CS0
- `/dev/ads1256_3.0`: SPI3, CS0
- `/dev/ads1256_4.0`: SPI4, CS0
- `/dev/ads1256_5.0`: SPI5, CS0
- `/dev/ads1256_6.0`: SPI6, CS0

### Writing MUX Configurations
Set the multiplexer (MUX) channels by writing an array of `struct ads1256_mux_sample` to the device file. The structure is defined as:
```
struct ads1256_mux_sample {
    unsigned char mux_config;    // MUX configuration (e.g., 0x0C for AIN0 vs. AINCOM)
    unsigned int value;          // Ignored when writing
    unsigned long long timestamp;// Ignored when writing
} __attribute__((packed));
```
**Example:** Configure AIN0 vs. AINCOM and AIN1 vs. AIN2:
```
struct ads1256_mux_sample configs[] = {
    {0x0C, 0, 0},  // AIN0 vs. AINCOM
    {0x12, 0, 0},  // AIN1 vs. AIN2
};
int fd = open("/dev/ads1256_0.0", O_RDWR);
write(fd, configs, sizeof(configs));
```
Valid MUX configurations include combinations of AIN0–AIN7 and AINCOM (e.g., `0x0C` for AIN0 vs. AINCOM, `0x12` for AIN1 vs. AIN2).
### Reading Samples
Read samples from the device file, receiving an array of `struct ads1256_mux_sample` with the MUX configuration, ADC value, and timestamp.Example:
```
struct ads1256_mux_sample samples[2];
read(fd, samples, sizeof(samples));
```
- Without buffering: Reads samples synchronously based on the MUX configurations.
- With buffering: Returns the latest complete cycle of samples from the circular buffer.

### IOCTL Commands
Configure the driver using `ioctl` commands:
- `ADS1256_SET_DATA_RATE`: Set sampling rate (e.g., 0x82 for 100 SPS).
```
int rate = 0x82;  // 100 SPS
ioctl(fd, ADS1256_SET_DATA_RATE, &rate);
```
- `ADS1256_SET_PGA_GAIN`: Set PGA gain (e.g., `0x00` for 1, `0x06` for 64).
```
int gain = 0x00;  // Gain 1
ioctl(fd, ADS1256_SET_PGA_GAIN, &gain);
```
- `ADS1256_SET_SENSOR_DETECT`: Set sensor detection current (e.g., `0x08` for 0.5 µA).
```
int sdcs = 0x08;  // 0.5 µA
ioctl(fd, ADS1256_SET_SENSOR_DETECT, &sdcs);
```
- `ADS1256_SET_BUFFERING`: Enable/disable buffering (1 = enable, 0 = disable).
```
int enable = 1;
ioctl(fd, ADS1256_SET_BUFFERING, &enable);
```
### Synchronization
Synchronize sampling across all devices by writing to the `sync` sysfs attribute of any device:
```
echo 1 | sudo tee /sys/class/ads1256/ads1256_0.0/sync
```
This triggers a shared sync GPIO pulse, aligning sampling across all ADS1256 instances.

### Buffering
Enable buffering to collect samples continuously into a circular buffer:
```
int enable = 1;
ioctl(fd, ADS1256_SET_BUFFERING, &enable);
```
- Buffer size is configurable via the buffer_size module parameter (default 1024).
- Read operations return the latest complete cycle of samples matching the MUX configuration count.

### Device Tree Configuration
The `ads1256-overlay.dts` file defines SPI and GPIO configurations. Customize GPIO pins using overlay parameters:
- **DRDY Pin:** e.g., `drdy0_pin=17` for SPI0.
- **Sync Pin:** e.g., `sync_pin=16` (shared across devices).
- **Reset Pin:** e.g., `reset0_pin=18` for SPI0 (optional).

#### Example:
```
sudo dtoverlay ads1256 drdy0_pin=17 sync_pin=16
```
Edit `ads1256-overlay.dts` for permanent changes, then recompile and apply as above.

### Sysfs Attributes
Each device exposes attributes under `/sys/class/ads1256/ads1256_X.Y/`:
- `current_sample`: Read the last sample (e.g., cat `current_sample`).
- `last_sample`: Trigger and read a new sample.
- `calibrate`: Trigger self-calibration (e.g., `echo 1 > calibrate`).
- `sync`: Synchronize all devices (e.g., `echo 1 > sync`).
- `data_rate`: Get/set data rate (e.g., `echo 0x82 > data_rate`).
- `pga_gain`: Get/set PGA gain (e.g., `echo 0x00 > pga_gain`).
- `dropped_samples`: Number of samples dropped due to buffer overflow.

### Example Code
The `read_ads1256.c` program demonstrates driver usage:
- Sets a 100 SPS data rate.
- Configures MUX channels (e.g., AIN0 vs. AINCOM, AIN1 vs. AIN2).
- Enables buffering and reads samples continuously, converting raw values to voltages.

#### Build and Run:
```
gcc -o read_ads1256 read_ads1256.c -lm
sudo ./read_ads1256
```
#### Sample Output:
```
PGA Gain: 1
Sample 0: MUX 0x0C, Value = 123456 (0x0001E240), Voltage = 0.007354 V at 1234567890123 ns
Sample 1: MUX 0x12, Value = -65432 (0xFFFEFF98), Voltage = -0.003897 V at 1234567990123 ns
```
### Module Parameters
- `buffer_size`: Set buffer size in samples (default 1024, max 65536).
```
sudo insmod ads1256.ko buffer_size=2048
```
- `debug`: Enable debug logging (0 = off, 1 = on).
```
sudo insmod ads1256.ko debug=1
```

## Troubleshooting
- **Permission Errors:** Use `sudo` or adjust device file permissions (e.g., `chmod 666 /dev/ads1256*`).
- **SPI Issues:** Ensure SPI is enabled in `/boot/config.txt` and the overlay matches your hardware setup.
- **GPIO Conflicts:** Check that DRDY, sync, and reset pins are not used by other overlays or drivers.
- **No Samples:** Verify MUX configurations are written before reading.

## Contributing
Contributions are welcome! Please:
- Report issues on the GitHub Issues page.
- Submit pull requests with bug fixes or enhancements.
- Suggest features via GitHub Discussions.

## License
This driver is licensed under the GNU General Public License (GPL). See the LICENSE file for details.

## Acknowledgments
Texas Instruments for the ADS1256 datasheet and reference materials.
Raspberry Pi community for SPI and device tree documentation.

