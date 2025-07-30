#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>

#define ADS1256_IOC_MAGIC 'a'
#define ADS1256_SET_DATA_RATE _IOW(ADS1256_IOC_MAGIC, 1, int)

struct ads1256_mux_sample {
    unsigned char mux_config;
    unsigned int value;
    unsigned long long timestamp;  // Matches u64
} __attribute__((packed));

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

    // Define MUX configs (e.g., AIN0 vs. AINCOM, AIN1 vs. AIN2 differential)
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
        if (ret <= 0) {  // Check for 0 or error
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

    // Read values
    struct ads1256_mux_sample *results = malloc(buf_size);
    if (!results) {
        perror("Failed to allocate memory for results");
        close(fd);
        return 1;
    }
    ssize_t bytes_read = read(fd, results, buf_size);
    if (bytes_read != buf_size) {
        perror("Failed to read samples");
        free(results);
        close(fd);
        return 1;
    }

    // Print results
    for (size_t i = 0; i < num_configs; i++) {
        printf("MUX 0x%02x: Value = %d (0x%08x) at %llu ns\n",
            results[i].mux_config, (int32_t)results[i].value, results[i].value, results[i].timestamp);
    }

    free(results);
    close(fd);
    return 0;
}