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
    void begin() {
        // ftok to generate unique key
        key = ftok("/tmp/ebike-ipc", 999);
        if (key == -1) {
            perror("ftok failed");
            exit(1);
        }

        // shmget returns an identifier in shmid
        shmid = shmget(key, 1024, 0666 | IPC_CREAT);
        if (shmid == -1) {
            perror("shmget failed");
            exit(1);
        }

        // shmat to attach to shared memory
        shm = (SharedMemory*)shmat(shmid, NULL, 0);
        if (shm == (void*)-1) {
            perror("shmat failed");
            exit(1);
        }
    };

    void stop() {
        // Detach from shared memory
        shmdt(shm);

        // Destroy the shared memory
        shmctl(shmid, IPC_RMID, NULL);
    }

    void write(const char *format, ...) {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        strcpy(shm->dataForServer, buffer);
        sem_post(&shm->dataServerRead);
    }

    std::string read() {
        sem_wait(&shm->dataClientRead);
        return (std::string)shm->dataForClient;
    }

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