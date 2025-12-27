// C++ program to show the example of server application in
// socket programming
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>

#define BUF_SIZE 10240

class ServerSocket {
public:
    int createServerSocket(int PORT);
    int createClientSocket();
    std::string read();
    int write(const char* data, size_t size);
    void stop();

    std::string receivedBuffer;
    int clientSocket = -1;
    int serverSocket = -1;
    int receivedLength = 0;
    bool isShutdown = 0;
};