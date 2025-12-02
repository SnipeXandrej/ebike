#include "ipcServer.hpp"

int IPCServer::begin() {
    system("/bin/bash -c '! [[ -e /tmp/ebike-ipc ]] && touch /tmp/ebike-ipc fi'");

    key = ftok("/tmp/ebike-ipc", 999);
    if (key == (key_t)-1) {
        perror("ftok failed");
        return -1;
    }

    shmid = shmget(key, sizeof(SharedMemory), 0666 | IPC_CREAT);
    if (shmid == -1) {
        perror("shmget failed");
        return -1;
    }

    shm = (SharedMemory*)shmat(shmid, NULL, 0);
    if (shm == (void*)-1) {
        perror("shmat failed");
        return -1;
    }

    // initialize contents and semaphores only once
    memset(shm, 0, sizeof(SharedMemory));

    sem_init(&shm->dataServerWrite, 1, 1);
    sem_init(&shm->clientCanRead,   1, 0);

    sem_init(&shm->dataClientWrite, 1, 1);
    sem_init(&shm->serverCanRead,   1, 0);

    shm->dataForServerLen.store(0, std::memory_order_relaxed);
    shm->dataForClientLen.store(0, std::memory_order_relaxed);

    shm->initialized.store(1, std::memory_order_release);

    return 0;
};

void IPCServer::stop() {
    // Detach from shared memory
    shmdt(shm);

    // Destroy the shared memory
    shmctl(shmid, IPC_RMID, NULL);
}

void IPCServer::write(const char* data, size_t size) {
    if (size > (sizeof(shm->dataForClient)))
        size = sizeof(shm->dataForClient);

    sem_wait(&shm->dataServerWrite);

    memcpy(shm->dataForClient, data, size);

    shm->dataForClientLen.store((int)size, std::memory_order_relaxed);

    sem_post(&shm->clientCanRead);
}

std::string IPCServer::read() {
    sem_wait(&shm->serverCanRead);

    int len = shm->dataForServerLen.load(std::memory_order_acquire);

    if (len < 0)
        len = 0;

    if ((size_t)len > sizeof(shm->dataForServer))
        len = sizeof(shm->dataForServer);

    std::string out(shm->dataForServer, (size_t)len);

    sem_post(&shm->dataClientWrite);

    return out;
}