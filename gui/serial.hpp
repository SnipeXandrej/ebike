#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

auto startLastTransmitTime = std::chrono::steady_clock::now();
auto endLastTransmitTime = std::chrono::steady_clock::now();

class SerialProcessor {
public:
    struct termios tty;

    std::string log;

    bool sent_once = false;

    bool succesfulCommunication = false;
    bool wasAllDataReceived = true;
    bool allDataReceived = true;
    int serialPort = -1;
    const char *serialPortNameStored;
    // char buffer[1024];            // Buffer for incoming data

    char bufferToRead[1024];            // Buffer for incoming data
    std::string receivedDataToRead;

    int timeout_ms = 5;

    float receiveRateMs = 0;
    int bytesInBuffer = 0;

    void init(const char *portName) {
        serialPortNameStored = portName;
        // std::cout << "return port 1: " << *serialPort << "\n";
        if (serialPort == -1 || !succesfulCommunication) {
            close(serialPort);

            serialPort = open(serialPortNameStored, O_RDWR | O_NOCTTY | O_NDELAY); //O_SYNC // NO_BLOCK???
            // std::cout << "return port 2: " << *serialPort << "\n";

            // Configure the serial port
            memset(&tty, 0, sizeof(tty));

            if (tcgetattr(serialPort, &tty) != 0) {
                // std::cerr << "[SERIAL] Error: Unable to get serial attributes.\n";
                log.append("Error: Unable to get serial attributes.\n");
                close(serialPort);
                // return 1;
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
                // std::cerr << "[SERIAL] Error: Unable to set serial attributes.\n";
                log.append("Error: Unable to set serial attributes.\n");
                close(serialPort);
                // return 1;
            }

            // std::cout << "return port 3: " << *serialPort << "\n";
        } else {
            succesfulCommunication = true;
        }
    }

    void readSerial(std::function<void(std::string)> execFunc) {
        // char bufferNext[1024];            // Buffer for incoming data
        std::string receivedData;    // String to store received lines

        // wasAllDataReceived = false;
        // if (!wasAllDataReceived) {
        //
        // }
        // ssize_t bytesReadNext;
        // ssize_t bytesRead; // = read(serialPort, buffer, sizeof(buffer) - 1); // Read data

        // memset(buffer, 0, sizeof(buffer)); // Initialize buffer
        // bytesRead = read(serialPort, buffer, sizeof(buffer) - 1);
        int bytes_available = 0;
        int bytes_available_previous = -1;
        ioctl(serialPort, FIONREAD, &bytes_available);

        while (bytes_available_previous != bytes_available) {
            bytes_available_previous = bytes_available;

            std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
            ioctl(serialPort, FIONREAD, &bytes_available);
        }

        bytesInBuffer = bytes_available;

        if (bytes_available > 0) {
            // std::cout << bytes_available << " byte(s) available\n";
            char buffer[bytes_available + 1];
            int n = read(serialPort, buffer, bytes_available);
            buffer[n] = '\0';
            // std::cout << "Received: " << buffer << std::endl;


            receivedData = buffer;
            receivedDataToRead = receivedData;

            endLastTransmitTime = std::chrono::steady_clock::now();
            receiveRateMs = std::chrono::duration<double, std::milli>(endLastTransmitTime - startLastTransmitTime).count();
            startLastTransmitTime = std::chrono::steady_clock::now();
            // std::cout << "receiveRateMs: " << receiveRateMs << "\n";
        }
        // bytesRead = read(serialPort, buffer, sizeof(buffer));
        // if (bytesRead > 0) {
        //     for (int i = 0; i < bytesRead; ++i) {
        //         char c = buffer[i];
        //         receivedData += c;
        //         if (c == '\n') {
        //             std::cout << "Received: " << receivedData;
        //             receivedDataToRead = receivedData;
        //             // receivedData.clear();
        //         }
        //     }
        // }


        if (bytes_available > 0) {
            // buffer[bytesRead] = '\0'; // Null-terminate the received data

            // Append to the string and check for complete lines
            // receivedData.append("\0");

            // Process each complete line
            size_t newlinePos;
            while ((newlinePos = receivedData.find('\n')) != std::string::npos) {
                std::string line = receivedData.substr(0, newlinePos);
                receivedData.erase(0, newlinePos + 1); // Remove processed line
                // std::cout << "line: " << line << "\n";

                // Remove any trailing '\r' (carriage return)
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                execFunc(line);
            }
            // std::cout << "\n";
            } else {
                // Error occurred
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // No data available (non-blocking)
                } else if (errno == EIO) {
                    // I/O error
                    // printf("[SERIAL] I/O error\n");
                    SerialProcessor::init(serialPortNameStored);
                    succesfulCommunication = false;
                    // std::cout << "[SERIAL] " << errno << " <-- errno\n";
                    log.append("{} <-- errno\n", errno);
                } else {
                    // printf("[SERIAL] Read error\n");
                    SerialProcessor::init(serialPortNameStored);
                    succesfulCommunication = false;
                    // std::cout << "[SERIAL] " << errno << " <-- errno\n";
                    log.append("{} <-- errno\n", errno);
                }
            }
        // } else if (bytesRead == -1) {
        //     // nothing was read
        //     // std::cerr << "Error: Failed to read from serial port.\n";
        //     // break;
        // }
    }

    void writeSerial(const char *data) {
            ssize_t bytesWritten = write(serialPort, data, strlen(data));
            if (bytesWritten == -1) {
                // std::cerr << "[SERIAL] Error: Failed to write to serial port.\n";
                log.append("Error: Failed to write to serial port.\n");
                // std::cout << "return port 4: " << serialPort << "\n";
            } else {
                // std::cout << "Sent: " << data << "\n";
            }
    }
};