// C++ program to illustrate the client application in the
// socket programming
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>

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

private:
    bool isShutdown = false;
};