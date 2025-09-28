#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

auto startLastTransmitTime = std::chrono::steady_clock::now();
auto endLastTransmitTime = std::chrono::steady_clock::now();

auto startMessageTimeout = std::chrono::steady_clock::now();
auto endMessageTimeout = std::chrono::steady_clock::now();

class SerialProcessor {
public:
    struct termios tty;

    std::string log;

    bool succesfulCommunication = false;
    int serialPort = -1;
    const char *serialPortNameStored;

    std::string receivedDataToRead;

    int timeout_ms = 5;

    float receiveRateMs = 0;
    float messageTimeout = 0;
    int bytesInBuffer = 0;

    void init(const char *portName) {
        serialPortNameStored = portName;
        if (serialPort == -1 || !succesfulCommunication) {
            close(serialPort);

            serialPort = open(serialPortNameStored, O_RDWR | O_NOCTTY | O_NDELAY); //O_SYNC // NO_BLOCK???

            // Configure the serial port
            memset(&tty, 0, sizeof(tty));

            if (tcgetattr(serialPort, &tty) != 0) {
                log.append("Error: Unable to get serial attributes.\n");
                close(serialPort);
            }

            cfsetospeed(&tty, B230400);  // Set baud rate to 4800 (output speed)
            cfsetispeed(&tty, B230400);  // Set baud rate to 4800 (input speed)

            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 data bits
            tty.c_iflag &= ~IGNBRK;                     // Ignore break processing
            tty.c_lflag = 0;                            // No signaling chars, no echo
            tty.c_oflag = 0;                            // No remapping, no delays
            tty.c_cc[VMIN] = 1;                         // Read at least 1 character
            tty.c_cc[VTIME] = 1;                        // Timeout in deciseconds

            if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
                log.append("Error: Unable to set serial attributes.\n");
                close(serialPort);
            }

        } else {
            succesfulCommunication = true;
        }
    }

    void readSerial(std::function<void(std::string)> execFunc) {
        std::string receivedData;    // String to store received lines

        int bytes_available = -1;
        ioctl(serialPort, FIONREAD, &bytes_available);

        std::string buffer;
        startMessageTimeout = std::chrono::steady_clock::now();
        messageTimeout = 0;
        while (bytes_available == 0 && messageTimeout < 500.0) {
            ioctl(serialPort, FIONREAD, &bytes_available);

            endMessageTimeout = std::chrono::steady_clock::now();
            messageTimeout = std::chrono::duration<double, std::milli>(endMessageTimeout - startMessageTimeout).count();
        }

        startMessageTimeout = std::chrono::steady_clock::now();
        messageTimeout = 0;
        while (bytes_available > 0 && messageTimeout < 500.0) {
            std::vector<char> buffer_TEMP(bytes_available);
            int n = read(serialPort, buffer_TEMP.data(), bytes_available);
            // std::cout << "buffer_TEMP: \n" << std::string(buffer_TEMP.data(), n) << "\n\n";

            buffer.append(buffer_TEMP.data(), n);

            std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
            ioctl(serialPort, FIONREAD, &bytes_available);

            endMessageTimeout = std::chrono::steady_clock::now();
            messageTimeout = std::chrono::duration<double, std::milli>(endMessageTimeout - startMessageTimeout).count();
        }

        if (buffer.size() > 0) {
            buffer += '\0';

            bytesInBuffer = buffer.size();

            receivedData = buffer;
            receivedDataToRead = receivedData;

            endLastTransmitTime = std::chrono::steady_clock::now();
            receiveRateMs = std::chrono::duration<double, std::milli>(endLastTransmitTime - startLastTransmitTime).count();
            startLastTransmitTime = std::chrono::steady_clock::now();


            // Process each complete line
            size_t newlinePos;
            while ((newlinePos = receivedData.find('\n')) != std::string::npos) {
                std::string line = receivedData.substr(0, newlinePos);
                receivedData.erase(0, newlinePos + 1); // Remove processed line

                // Remove any trailing '\r' (carriage return)
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                execFunc(line);
            }
        } else {
            // Error occurred
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available (non-blocking)
            } else if (errno == EIO) {
                // I/O error
                SerialProcessor::init(serialPortNameStored);
                succesfulCommunication = false;
                log.append("{} <-- errno\n", errno);
            } else {
                SerialProcessor::init(serialPortNameStored);
                succesfulCommunication = false;
                log.append("{} <-- errno\n", errno);
            }
        }
    }

    void writeSerial(const char *data) {
            ssize_t bytesWritten = write(serialPort, data, strlen(data));
            if (bytesWritten == -1) {
                log.append("Error: Failed to write to serial port.\n");
            } else {
                // std::cout << "Sent: " << data << "\n";
            }
    }
};