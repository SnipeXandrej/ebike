#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <netdb.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "server.hpp"
#include "messagingUtils.hpp"

bool TcpServer::ready() const {
    return listener_.load() >= 0;
}

bool TcpServer::serve(const MessageHandler &on_message) {
    while (running_) {
        sockaddr_storage address{};
        socklen_t address_size = sizeof(address);
        int fd = accept(listener_.load(),
                        reinterpret_cast<sockaddr *>(&address),
                        &address_size);
        if (fd < 0) {
            if (errno == EINTR)
                continue;
            return !running_;
        }

        auto client = std::make_shared<Client>();
        client->fd = fd;
        client->id = next_id_++;
        client->name = endpoint_name(fd,
                                        reinterpret_cast<sockaddr *>(&address),
                                        address_size);
        {
            std::lock_guard lock(clients_mutex_);
            clients_.push_back(client);
        }
        std::fprintf(stderr, "Client %lu connected: %s\n", client->id,
                        client->name.c_str());

        try {
            std::lock_guard lock(receivers_mutex_);
            receivers_.emplace_back(&TcpServer::receive_loop, this, client,
                                    on_message);
        } catch (...) {
            remove(client);
            close(fd);
        }
    }
    return true;
}

bool TcpServer::send_to(unsigned long id, const char *data, size_t size) {
    std::shared_ptr<Client> destination; {
        std::lock_guard lock(clients_mutex_);
        for (const auto &client : clients_)
            if (client->id == id) {
                destination = client;
                break;
            }
    }

    if (!destination)
        return false;

    std::lock_guard lock(destination->send_mutex);

    if (!send_all(destination->fd, data, size))
        shutdown(destination->fd, SHUT_RDWR);

    return true;
}

std::vector<TcpServer::ClientInfo> TcpServer::clients() const {
    std::vector<ClientInfo> result;
    std::lock_guard lock(clients_mutex_);
    for (const auto &client : clients_)
        result.push_back({client->id, client->name});
    return result;
}

void TcpServer::shutdown_writes() {
    std::lock_guard lock(clients_mutex_);
    for (const auto &client : clients_)
        shutdown(client->fd, SHUT_WR);
}

void TcpServer::stop() {
    if (!running_.exchange(false))
        return;

    int listener = listener_.exchange(-1);
    if (listener >= 0) {
        shutdown(listener, SHUT_RDWR);
        close(listener);
    }

    std::lock_guard lock(clients_mutex_);
    for (const auto &client : clients_)
        shutdown(client->fd, SHUT_RDWR);
}

bool TcpServer::send_all(int fd, const char *data, size_t size) {
    while (size > 0) {
        ssize_t sent = send(fd, data, size, 0);
        if (sent < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        data += sent;
        size -= static_cast<size_t>(sent);
    }
    return true;
}

int TcpServer::create_listener(const char *port) {
    addrinfo hints{};
    addrinfo *addresses = nullptr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    if (getaddrinfo(nullptr, port, &hints, &addresses) != 0)
        return -1;

    int listener = -1;
    int reuse = 1;
    for (addrinfo *address = addresses; address != nullptr;
            address = address->ai_next) {
        listener = socket(address->ai_family, address->ai_socktype,
                            address->ai_protocol);

        if (listener < 0)
            continue;

        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        if (bind(listener, address->ai_addr, address->ai_addrlen) == 0 &&
            listen(listener, BACKLOG) == 0)
            break;

        close(listener);
        listener = -1;
    }
    freeaddrinfo(addresses);
    return listener;
}

std::string TcpServer::endpoint_name(int fd, const sockaddr *address, socklen_t size) {
    char host[64];
    char service[16];
    if (getnameinfo(address, size, host, sizeof(host), service,
                    sizeof(service), NI_NUMERICHOST | NI_NUMERICSERV) == 0)
        return std::string(host) + ":" + service;
    return "fd-" + std::to_string(fd);
}

void TcpServer::receive_loop(std::shared_ptr<Client> client, const MessageHandler &on_message) {
    char buffer[BUFFER_SIZE];
    std::string received_buffer;

    for (;;) {
        ssize_t received = recv(client->fd, buffer, sizeof(buffer), 0);
        if (received == 0)
            break;
        if (received < 0) {
            if (errno == EINTR)
                continue;
            break;
        }

        received_buffer.append(buffer, static_cast<size_t>(received));

        size_t consumed = 0;
        while (true) {
            const size_t end_pos = received_buffer.find(msg::messageEnd, consumed);
            if (end_pos == std::string::npos)
                break;
            consumed = end_pos + msg::messageEnd.size();
        }

        if (consumed > 0) {
            on_message(client->id, client->name, received_buffer.data(), consumed);
            received_buffer.erase(0, consumed);
        }
    }
    remove(client);
    close(client->fd);
    std::fprintf(stderr, "Client disconnected: %s\n", client->name.c_str());
}

void TcpServer::remove(const std::shared_ptr<Client> &target) {
    std::lock_guard lock(clients_mutex_);
    for (auto current = clients_.begin(); current != clients_.end();
            ++current)
        if (*current == target) {
            clients_.erase(current);
            return;
        }
}

void TcpServer::wait_for_receivers() {
    std::vector<std::thread> receivers;
    {
        std::lock_guard lock(receivers_mutex_);
        receivers.swap(receivers_);
    }

    for (auto &receiver : receivers)
        if (receiver.joinable())
            receiver.join();
}