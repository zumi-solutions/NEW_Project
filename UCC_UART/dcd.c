#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int main()
{
    int fd;
    int status;
    int last_dcd = -1;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Open failed");
        return 1;
    }

    printf("Monitoring DCD pin... Press Ctrl+C to exit\n");

    while (1)
    {
        if (ioctl(fd, TIOCMGET, &status) == -1) {
            perror("TIOCMGET failed");
            close(fd);
            return 1;
        }

        int dcd = (status & TIOCM_CAR) ? 1 : 0;

        if (dcd != last_dcd) {
            if (dcd)
                printf("DCD is HIGH (Carrier Detected)\n");
            else
                printf("DCD is LOW (No Carrier)\n");

            last_dcd = dcd;
        }

        usleep(200000);  // 200 ms delay
    }

    close(fd);
    return 0;
}
