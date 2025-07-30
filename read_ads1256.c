#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <poll.h>

#define ADS1256_IOC_MAGIC 'a'
#define ADS1256_SET_DATA_RATE _IOW(ADS1256_IOC_MAGIC, 1, int)
#define ADS1256_SET_BUFFERING _IOW(ADS1256_IOC_MAGIC, 7, int)

#define VREF 2.5  // Reference voltage in volts (adjust as needed)
#define FULL_SCALE 0x7FFFFF  // 2^23 - 1, max positive value for 24-bit signed ADC

struct ads1256_mux_sample {
    unsigned char mux_config;
    unsigned int value;
    unsigned long long timestamp;  // Matches u64
} __attribute__((packed));

// Function to read PGA gain from sysfs
int read_pga_gain(void) {
    FILE *f = fopen("/sys/class/ads1256/ads1256_0.0/pga_gain", "r");
    if (!f) {
        perror("Failed to open pga_gain sysfs");
        return -1;  // Error case
    }
    char buf[16];
    if (!fgets(buf, sizeof(buf), f)) {
        perror("Failed to read pga_gain");
        fclose(f);
        return -1;
    }
    fclose(f);
    unsigned int pga_hex;
    if (sscanf(buf, "0x%x", &pga_hex) != 1) {
        fprintf(stderr, "Invalid pga_gain format: %s\n", buf);
        return -1;
    }
    // Convert PGA hex to actual gain (e.g., 0x00 -> 1, 0x01 -> 2, etc.)
    switch (pga_hex) {
        case 0x00: return 1;
        case 0x01: return 2;
        case 0x02: return 4;
        case 0x03: return 8;
        case 0x04: return 16;
        case 0x05: return 32;
        case 0x06: return 64;
        default:
            fprintf(stderr, "Unknown PGA gain value: 0x%x\n", pga_hex);
            return -1;
    }
}

// Function to convert raw ADC value to voltage
double raw_to_voltage(int32_t raw, int pga) {
    // Sign-extend the 24-bit value (already done in driver, but verify)
    if (raw & 0x800000) {
        raw |= 0xFF000000;  // Extend sign bit to 32 bits
    }
    // Convert to voltage: V = (Raw * Vref) / (PGA * Full-Scale)
    return (double)raw * VREF / (pga * FULL_SCALE);
}

int main() {
    int fd = open("/dev/ads1256_0.0", O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    // Set data rate to 100 SPS
    int new_rate = 0x82;  // ADS1256_DRATE_SPS_100
    if (ioctl(fd, ADS1256_SET_DATA_RATE, &new_rate) < 0) {
        perror("Failed to set data rate");
        close(fd);
        return 1;
    }

    // Define MUX configs
    struct ads1256_mux_sample configs[] = {
        {0x0C, 0, 0},  // AIN0 vs. AINCOM (single-ended)
        {0x12, 0, 0},  // AIN1 vs. AIN2 (differential)
        {0x34, 0, 0},  // AIN3 vs. AIN4 (differential)
        {0x3C, 0, 0},  // AIN3 vs. AINCOM (single-ended)
    };
    size_t num_configs = sizeof(configs) / sizeof(configs[0]);
    size_t buf_size = num_configs * sizeof(struct ads1256_mux_sample);

    // Write MUX configs
    ssize_t written = 0;
    int retries = 5;
    while (written < buf_size && retries--) {
        ssize_t ret = write(fd, configs + written / sizeof(struct ads1256_mux_sample), buf_size - written);
        if (ret <= 0) {
            if (ret < 0 && errno == EINTR)
                continue;
            fprintf(stderr, "Failed to write MUX configs: %s (ret=%zd, errno=%d)\n",
                    strerror(errno), ret, errno);
            close(fd);
            return 1;
        }
        written += ret;
    }
    if (written < buf_size) {
        fprintf(stderr, "Incomplete write: %zd of %zu bytes written\n", written, buf_size);
        close(fd);
        return 1;
    }

    // Enable buffering
    int enable = 1;
    if (ioctl(fd, ADS1256_SET_BUFFERING, &enable) < 0) {
        perror("Failed to enable buffering");
        close(fd);
        return 1;
    }

    // Read PGA gain from sysfs
    int pga = read_pga_gain();
    if (pga < 0) {
        fprintf(stderr, "Using default PGA gain of 1\n");
        pga = 1;  // Fallback to 1 if sysfs read fails
    }
    printf("PGA Gain: %d\n", pga);

    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    while (1) {
        if (poll(&pfd, 1, -1) < 0) {  // Wait for samples
            perror("Poll failed");
            close(fd);
            return 1;
        }
        struct ads1256_mux_sample samples[10];
        ssize_t bytes = read(fd, samples, sizeof(samples));
        if (bytes < 0) {
            if (errno == EAGAIN) {
                continue;  // No data yet, keep polling
            }
            perror("Read failed");
            close(fd);
            return 1;
        }
        if (bytes > 0) {
            for (int i = 0; i < bytes / sizeof(samples[0]); i++) {
                double voltage = raw_to_voltage(samples[i].value, pga);
                printf("Sample %d: MUX 0x%02x, Value = %d (0x%08x), Voltage = %.6f V at %llu ns\n",
                       i, samples[i].mux_config, (int32_t)samples[i].value, samples[i].value, voltage, samples[i].timestamp);
            }
        }
    }

    close(fd);
    return 0;
}