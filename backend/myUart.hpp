#ifndef MYUART_HPP
#define MYUART_HPP

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>


class MyUart {
public:
    int begin(speed_t BAUDRATE);

    void print(const char* data);

    void printf(const char* format, ...);

    int read();

    int available();

    void write(const uint8_t *data, size_t len);

    int fd;
};

#endif