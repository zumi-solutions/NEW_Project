#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

int main()
{
    int fd;
    int status;
    int last_cts = -1;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Open failed");
        return 1;
    }

    printf("Monitoring CTS pin... Press Ctrl+C to exit\n");

    while (1)
    {
        if (ioctl(fd, TIOCMGET, &status) == -1) {
            perror("TIOCMGET failed");
            close(fd);
            return 1;
        }

        int cts = (status & TIOCM_CTS) ? 1 : 0;

        if (cts != last_cts) {
            if (cts)
                printf("CTS is HIGH (Active)\n");
            else
                printf("CTS is LOW (Inactive)\n");

            last_cts = cts;
        }

        usleep(200000); // 200ms delay
    }

    close(fd);
    return 0;
}
