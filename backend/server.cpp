#include "server.hpp"
#include "messagingUtils.hpp"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>

int ServerSocket::createServerSocket(int PORT) {
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);

    struct timeval tv;
    tv.tv_sec = connectionTimeoutS;
    tv.tv_usec = 0;
    setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(serverSocket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cout << "[IPC] Another server is running, or the port is being used by another process\n";
        return -1;
    }

    if (listen(serverSocket, SOMAXCONN) == -1) {
        perror("listen");
        return -1;
    }

    return 0;
}

int ServerSocket::createClientSocket() {
    clientSocket = accept(serverSocket, nullptr, nullptr);
    if (clientSocket < 0) {
        return -1;
    }

    receivedBuffer.clear();
    receivedLength = 0;

    std::cout << "[IPC] Client connected!\n";
    return 0;
}

std::string ServerSocket::read() {
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
        char buffer[10240];
        int len = recv(clientSocket, buffer, sizeof(buffer), 0);

        if (len > 0) {
            receivedBuffer.append(buffer, len);
        } else if (len <= 0) {
            std::cout << "[IPC] client disconnected\n";
            shutdown(clientSocket, SHUT_RDWR);
            close(clientSocket);

            if (isShutdown) {
                return "";
            }

            // Accept a new client
            while (createClientSocket() < 0 && !isShutdown) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::printf("retrying connection\n");
            }

        }
    }
}

int ServerSocket::write(const char* data, size_t size) {
    if (clientSocket < 0) return -1;

    int ret = send(clientSocket, data, size, MSG_NOSIGNAL);
    if (ret < 0) perror("[IPC] send");

    return ret;
}

void ServerSocket::stop() {
    shutdown(clientSocket, SHUT_RDWR);
    close(clientSocket);
    shutdown(serverSocket, SHUT_RDWR);
    close(serverSocket);
    isShutdown = true;
}
