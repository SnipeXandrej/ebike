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
    void write(const char *format, ...);
    std::string read();
    bool successfulCommunication = false;

private:
    struct SharedMemory {
        sem_t dataServerRead;
        sem_t dataClientRead;
        char dataForServer[1024];
        char dataForClient[1024];
    };

    SharedMemory* shm;
    // int shmid;
    // key_t key;
};

#endif