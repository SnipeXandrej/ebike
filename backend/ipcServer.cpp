#include "ipcServer.hpp"

int IPCServer::begin() {
    system("/bin/bash -c '! [[ -e /tmp/ebike-ipc ]] && touch /tmp/ebike-ipc fi'");

    // ftok to generate unique key
    key = ftok("/tmp/ebike-ipc", 999);
    if (key == -1) {
        perror("ftok failed");
        return -1;
    }

    // shmget returns an identifier in shmid
    shmid = shmget(key, 1024, 0666 | IPC_CREAT);
    if (shmid == -1) {
        perror("shmget failed");
        return -1;
    }

    // shmat to attach to shared memory
    shm = (SharedMemory*)shmat(shmid, NULL, 0);
    if (shm == (void*)-1) {
        perror("shmat failed");
        return -1;
    }

    strcpy(shm->dataForServer, "");
    strcpy(shm->dataForClient, "");

    sem_init(&shm->dataServerRead, 1, 0);
    sem_init(&shm->dataClientRead, 1, 0);

    sem_post(&shm->dataClientRead);
    sem_post(&shm->dataServerRead);

    return 0;
};

void IPCServer::stop() {
    // Detach from shared memory
    shmdt(shm);

    // Destroy the shared memory
    shmctl(shmid, IPC_RMID, NULL);
}

void IPCServer::write(const char *format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    strcpy(shm->dataForClient, buffer);
    sem_post(&shm->dataClientRead);
}

std::string IPCServer::read() {
    sem_wait(&shm->dataServerRead);
    return (std::string)shm->dataForServer;
}