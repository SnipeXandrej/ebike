#ifndef IPCSERVER_H
#define IPCSERVER_H

#include <iostream>
#include <cstring>
#include <thread>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <stdarg.h>

class IPCServer {
public:
    void begin();
    void stop();
    void write(const char *format, ...);
    std::string read();

private:
    struct SharedMemory {
        sem_t dataServerRead;
        sem_t dataClientRead;
        char dataForServer[1024];
        char dataForClient[1024];
    };

    SharedMemory* shm;
    int shmid;
    key_t key;
};

#endif