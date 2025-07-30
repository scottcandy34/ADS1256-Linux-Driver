#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>  // Added for int32_t

int main() {
    int fd = open("/dev/ads1256", O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device /dev/ads1256");
        return 1;
    }

    int32_t sample;  // Changed to int32_t for signed values
    while (1) {  // Added loop for continuous testing
        ssize_t bytes_read = read(fd, &sample, sizeof(sample));
        if (bytes_read == sizeof(sample)) {
            printf("ADC Value: %d (0x%08x)\n", sample, (uint32_t)sample);  // Updated format for signed value
        } else if (bytes_read < 0) {
            perror("Failed to read from device");  // Simplified error handling
            close(fd);
            return 1;
        } else {
            fprintf(stderr, "Incomplete read: expected %zu bytes, got %zd bytes\n", sizeof(sample), bytes_read);
            close(fd);
            return 1;
        }
        sleep(1);  // Added delay for continuous reading
    }

    if (close(fd) < 0) {
        perror("Failed to close device");
        return 1;
    }
    return 0;
}