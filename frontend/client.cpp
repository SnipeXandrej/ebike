#include "client.hpp"

int ClientSocket::createClientSocket(int PORT) {
    return createClientSocket(PORT, "0.0.0.0");
}

int ClientSocket::createClientSocket(int PORT, const char* ADDRESS) {
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    inet_pton(AF_INET, ADDRESS, &serverAddress.sin_addr);

    int ret = connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
    if (ret == -1) {
        std::cout << "[IPC] Another server is running, or the port is being used by another process\n";
        exit(EXIT_FAILURE);
    }

    return 0;
}

std::string ClientSocket::read() {
    std::string output;

    while (true) {
        // Check if there is at least one complete line in the buffer
        size_t newlinePos = receivedBuffer.find('\n');
        if (newlinePos != std::string::npos) {
            // get all complete lines
            size_t pos = 0;
            while (true) {
                newlinePos = receivedBuffer.find('\n', pos);
                if (newlinePos == std::string::npos) break;

                // include '\n' in the extracted line (+1)
                std::string line = receivedBuffer.substr(pos, newlinePos - pos + 1);

                output += line;
                pos = newlinePos + 1;
            }

            // Remove processed lines from the buffer
            receivedBuffer.erase(0, pos);

            receivedLength = output.size();

            return output;  // return all complete lines
        }

        // if theres no complete line then try receiving more data
        char buffer[BUF_SIZE];
        int len = recv(clientSocket, buffer, sizeof(buffer), 0);

        if (len == -1) {
            clientSocket = createClientSocket(8080);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (len == 0) {
            std::cout << "[IPC] client disconnected or connection closed\n";
            close(clientSocket);
            exit(EXIT_FAILURE);
        }

        receivedBuffer.append(buffer, len);
    }
}

int ClientSocket::write(const char* data, size_t size) {
    int ret = send(clientSocket, data, size, MSG_NOSIGNAL);

    return ret;
}

void ClientSocket::stop() {
    close(clientSocket);
}