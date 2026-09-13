#pragma once

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

class TcpServer {
  public:
    struct ClientInfo {
      unsigned long id;
      std::string name;
    };

    using MessageHandler = std::function<void(unsigned long, const std::string&, const char*, size_t)>;

    explicit TcpServer(const char *port) : listener_(create_listener(port)), running_(listener_.load() >= 0) {
        //
    }

    ~TcpServer() {
        stop();
        wait_for_receivers();
    };
    bool ready() const;
    bool serve(const MessageHandler &on_message);
    bool send_to(unsigned long id, const char *data, size_t size);
    std::vector<ClientInfo> clients() const;
    void shutdown_writes();
    void stop();

  private:
    struct Client {
        int fd;
        unsigned long id;
        std::string name;
        std::mutex send_mutex;
    };

    static constexpr int BACKLOG = 16;
    static constexpr size_t BUFFER_SIZE = 4096;

    static bool send_all(int fd, const char *data, size_t size);
    static int create_listener(const char *port);
    static std::string endpoint_name(int fd, const sockaddr *address, socklen_t size);
    void receive_loop(std::shared_ptr<Client> client, const MessageHandler &on_message);
    void remove(const std::shared_ptr<Client> &target);
    void wait_for_receivers();

    std::atomic<int> listener_;
    std::atomic<bool> running_;
    unsigned long next_id_ = 1;
    mutable std::mutex clients_mutex_;
    std::vector<std::shared_ptr<Client>> clients_;
    std::mutex receivers_mutex_;
    std::vector<std::thread> receivers_;
};
