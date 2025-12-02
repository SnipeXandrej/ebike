#include "ipcClient.hpp"

int IPCClient::begin() {
    key_t key = ftok("/tmp/ebike-ipc", 999);
    if (key == (key_t)-1) {
        perror("ftok failed");
        successfulCommunication = false;
        return -1;
    }

    int shmid = shmget(key, sizeof(SharedMemory), 0666);
    if (shmid == -1) {
        perror("shmget failed");
        successfulCommunication = false;
        return -1;
    }

    shm = (SharedMemory*)shmat(shmid, NULL, 0);
    if (shm == (void*)-1) {
        perror("shmat failed");
        successfulCommunication = false;
        return -1;
    }

    while (shm->initialized.load(std::memory_order_acquire) == 0) {
        usleep(1000);
    }

    successfulCommunication = true;
    return 0;
};

void IPCClient::stop() {
    // // Detach from shared memory
    shmdt(shm);

    // // Destroy the shared memory
    // shmctl(shmid, IPC_RMID, NULL);
}

void IPCClient::write(const char *data, size_t size) {
    if (size > (sizeof(shm->dataForServer)))
        size = sizeof(shm->dataForServer);

    sem_wait(&shm->dataClientWrite);

    memcpy(shm->dataForServer, data, size);

    shm->dataForServerLen.store((int)size, std::memory_order_relaxed);

    sem_post(&shm->serverCanRead);
}

std::string IPCClient::read() {
    sem_wait(&shm->clientCanRead);

    int len = shm->dataForClientLen.load(std::memory_order_acquire);

    if (len < 0)
        len = 0;

    if ((size_t)len > sizeof(shm->dataForClient))
        len = sizeof(shm->dataForClient);

    std::string out(shm->dataForClient, (size_t)len);

    sem_post(&shm->dataServerWrite);

    return out;
}