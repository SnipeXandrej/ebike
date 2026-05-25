#include "client.hpp"
#include "messagingUtils.hpp"
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>

bool previousConnection = false;

int ClientSocket::createClientSocket(int PORT) {
    return createClientSocket(PORT, "0.0.0.0");
}

int ClientSocket::createClientSocket(int PORT, const char* ADDRESS) {
    lastServerAddress = (char*)ADDRESS;
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);

    struct timeval tv;
    tv.tv_sec = connectionTimeoutS;
    tv.tv_usec = 0;
    setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(clientSocket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    int yes = 1;
    setsockopt(clientSocket, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));

    int flag = 1;
    setsockopt(clientSocket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    inet_pton(AF_INET, lastServerAddress, &serverAddress.sin_addr);

    int ret = connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
    if (ret == -1) {
        if (previousConnection) {
            previousConnection = false;
            std::cout << "[IPC] Another server is running, or the port is being used by another process, or is not running at all.. you guess\n";
        }
        isConnected = false;
        return -1;
    }

    receivedBuffer.clear();
    receivedLength = 0;

    previousConnection = true;
    isConnected = true;
    return 0;
}

std::string ClientSocket::read() {
    std::string output;

    while (true) {
        // Check if there is at least one complete line in the buffer
        // Complete lines end with msg::messageEnd ("@@!!\n")
        size_t consumed = 0;
        while (true) {
            size_t endPos = receivedBuffer.find(msg::messageEnd, consumed);
            if (endPos == std::string::npos)
                break;

            size_t frameEnd = endPos + msg::messageEnd.size();
            output.append(receivedBuffer, consumed, frameEnd - consumed);
            consumed = frameEnd;
        }

        if (consumed > 0) {
            receivedBuffer.erase(0, consumed);

            receivedLength = output.size();

            return output;
        }

        // if theres no complete line then try receiving more data
        char buffer[BUF_SIZE];
        int len = recv(clientSocket, buffer, sizeof(buffer), 0);

        if (len == -1) {
            if (!isShutdown) {
                shutdown(clientSocket, SHUT_RDWR);
                close(clientSocket);
                createClientSocket(8080, lastServerAddress);
            } else {
                return "";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (len == 0) {
            std::cout << "[IPC] client disconnected or connection closed\n";
            close(clientSocket);
            return "";
        }

        amountOfDataReceived += len;

        receivedBuffer.append(buffer, len);
    }
}

int ClientSocket::write(const char* data, size_t size) {
    while (!isConnected && !isShutdown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    int ret = send(clientSocket, data, size, MSG_NOSIGNAL);

    if (ret == -1) {
        isConnected = false;
        return -1;
    }

    amountOfDataSent += ret;
    return ret;
}

void ClientSocket::stop() {
    isConnected = false;
    shutdown(clientSocket, SHUT_RDWR);
    isShutdown = true;
    close(clientSocket);
}