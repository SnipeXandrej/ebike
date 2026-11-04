#ifndef MYUART_HPP
#define MYUART_HPP

#include <inttypes.h>
#include <stdio.h>
#include "hal.h"

class MyUart {
public:
    void setUartPort(SerialDriver* _port);

    int begin(int BAUDRATE);

    void print(const char* data);

    void printf(const char* format, ...);

    int read();

    int available();

    void write(const uint8_t *data, size_t len);

    int fd = -1;

private:
    SerialDriver* uartPort = nullptr;
};

#endif
