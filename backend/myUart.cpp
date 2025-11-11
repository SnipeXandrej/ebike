#include "myUart.hpp"

int MyUart::begin(speed_t BAUDRATE) {
    struct termios options;
    int status;

    if ((fd = open ("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1) {
        return -1;
    }

    fcntl (fd, F_SETFL, O_RDWR);

    tcgetattr (fd, &options);

    cfmakeraw   (&options);
    cfsetispeed (&options, BAUDRATE);
    cfsetospeed (&options, BAUDRATE);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    options.c_cc [VMIN]  =   0;
    options.c_cc [VTIME] = 100; // 10s

    tcsetattr (fd, TCSANOW, &options) ;

    ioctl (fd, TIOCMGET, &status);

    status |= TIOCM_DTR ;
    status |= TIOCM_RTS ;

    ioctl (fd, TIOCMSET, &status);

    usleep (10000) ;	// 10mS

    if (fd < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
        return -1;
    }

    return 0;
}

void MyUart::print(const char* data) {
    ::write(fd, (const char*)data, strlen(data));
}

void MyUart::printf(const char* format, ...) {
    ::write(fd, (const char*)format, strlen(format));
}

int MyUart::read() {
    uint8_t c = 0;

    ::read(fd, &c, 1);

    return c;
}

int MyUart::available() {
    size_t available;
    if (ioctl(fd, FIONREAD, &available) == -1) {
        return -1;
    }

    return available;
}

void MyUart::write(const uint8_t *data, size_t len) {
    if (data == NULL || !len) {
        return;
    }

    ::write(fd, data, len);
}