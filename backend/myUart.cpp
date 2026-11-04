#include "myUart.hpp"
#include "ch.h"
#include "chprintf.h"

#include <stdarg.h>
#include <string.h>

void MyUart::setUartPort(SerialDriver* _port) {
    uartPort = _port;
}

int MyUart::begin(int BAUDRATE) {
    if (BAUDRATE <= 0) {
        return -1;
    }

    const SerialConfig config = {
        static_cast<uint32_t>(BAUDRATE),
        0U,
        0U,
        0U
    };

    return static_cast<int>(sdStart(uartPort, &config));
}

void MyUart::print(const char* data) {
    if (data != nullptr) {
        chnWrite(uartPort, reinterpret_cast<const uint8_t *>(data), strlen(data));
    }
}

void MyUart::printf(const char* format, ...) {
    if (format == nullptr) {
        return;
    }

    va_list args;
    va_start(args, format);
    chvprintf(reinterpret_cast<BaseSequentialStream *>(uartPort), format, args);
    va_end(args);
}

int MyUart::read() {
    const msg_t value = chnGetTimeout(uartPort, TIME_INFINITE);
    return value == MSG_TIMEOUT ? -1 : static_cast<int>(value);
}

int MyUart::available() {
    size_t count;

    chSysLock();
    count = iqGetFullI(&uartPort->iqueue);
    chSysUnlock();

    return static_cast<int>(count);
}

void MyUart::write(const uint8_t *data, size_t len) {
    if ((data != nullptr) && (len != 0U)) {
        chnWrite(uartPort, data, len);
    }
}
