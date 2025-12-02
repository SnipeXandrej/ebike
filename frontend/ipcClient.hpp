#ifndef IPCCLIENT_H
#define IPCCLIENT_H

#include <iostream>
#include <cstring>
#include <thread>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <stdarg.h>

class IPCClient {
public:
    int begin();
    void stop();
    void write(const char *data, size_t size);
    std::string read();
    bool successfulCommunication = false;

private:
    struct SharedMemory {
        // server->client
        sem_t dataServerWrite;
        sem_t clientCanRead;

        // client->server
        sem_t dataClientWrite;
        sem_t serverCanRead;

        // payloads
        char dataForServer[4096];
        char dataForClient[4096];
        std::atomic<int> dataForServerLen;
        std::atomic<int> dataForClientLen;

        std::atomic<int> initialized;
    };

    SharedMemory* shm;
    // int shmid;
    // key_t key;
};

#endif