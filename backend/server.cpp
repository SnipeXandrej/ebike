#include "server.hpp"

int ServerSocket::createServerSocket(int PORT) {
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cout << "[IPC] Another server is running, or the port is being used by another process\n";
        exit(EXIT_FAILURE);
    }

    listen(serverSocket, 1);

    return 0;
}

int ServerSocket::createClientSocket() {
    clientSocket = accept(serverSocket, nullptr, nullptr);
    if (clientSocket < 0) {
        return -1;
    }

    std::cout << "[IPC] Client connected!\n";
    return 0;
}

std::string ServerSocket::read() {
    std::string output;

    while (true) {
        // Check if there is at least one complete line in the buffer
        size_t newlinePos = receivedBuffer.find('\n');
        if (newlinePos != std::string::npos) {
            size_t pos = 0;
            while (true) {
                newlinePos = receivedBuffer.find('\n', pos);
                if (newlinePos == std::string::npos) break;

                output += receivedBuffer.substr(pos, (newlinePos - pos) + 1);
                pos = newlinePos + 1;
            }

            receivedBuffer.erase(0, pos);
            receivedLength = output.size();

            return output;
        }

        // if theres no complete line then try receiving more data
        char buffer[10240];
        int len = recv(clientSocket, buffer, sizeof(buffer), 0);

        if (len > 0) {
            receivedBuffer.append(buffer, len);
        } else if (len == 0) {
            // client disconnected, create new connection
            std::cout << "[IPC] client disconnected\n";
            close(clientSocket);
            receivedBuffer.clear();
            receivedLength = 0;

            do {
                createClientSocket();
            } while (clientSocket < 0);
        } else if (len == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
    }
}

int ServerSocket::write(const char* data, size_t size) {
    int ret = send(clientSocket, data, size, MSG_NOSIGNAL);

    return ret;
}

void ServerSocket::stop() {
    close(clientSocket);
    close(serverSocket);
}
