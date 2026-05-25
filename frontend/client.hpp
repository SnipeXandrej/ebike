// C++ program to illustrate the client application in the
// socket programming
#include <cstdlib>
#include <iostream>

#define BUF_SIZE 10240

class ClientSocket {
public:
    int createClientSocket(int PORT);
    int createClientSocket(int PORT, const char* ADDRESS);
    int write(const char* data, size_t size);
    std::string read();
    void stop();

    int clientSocket = -1;
    int receivedLength = 0;
    std::string receivedBuffer;

    char* lastServerAddress;
    bool isConnected = false;

    uint64_t amountOfDataReceived = 0;
    uint64_t amountOfDataSent = 0;

private:
    bool isShutdown = false;
    int connectionTimeoutS = 30;
};