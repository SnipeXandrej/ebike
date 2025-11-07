#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <chrono>
#include <functional>
#include <cstring>
#include <thread>

class SerialProcessor {
public:
    struct termios tty;

    int         serialPort = -1;

    std::string log;
    std::string receivedDataToRead;

    bool        succesfulCommunication = false;
    const char  *serialPortNameStored;
    int         timeout_ms = 5;
    float       receiveRateMs = 0;
    float       messageTimeout = 0;
    int         bytesInBuffer = 0;

    void init(const char *portName);
    void readSerial(std::function<void(std::string)> execFunc);
    void writeSerial(const char *data);

    private:
        std::chrono::_V2::steady_clock::time_point startLastTransmitTime = std::chrono::steady_clock::now();
        std::chrono::_V2::steady_clock::time_point endLastTransmitTime = std::chrono::steady_clock::now();

        std::chrono::_V2::steady_clock::time_point startMessageTimeout = std::chrono::steady_clock::now();
        std::chrono::_V2::steady_clock::time_point endMessageTimeout = std::chrono::steady_clock::now();
};