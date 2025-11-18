#include "ipcClient.hpp"

int IPCClient::begin() {
    shmdt(shm);
    shm = nullptr;

    // std::printf("shm3: %d\n", (void*)shm);
    // ftok to generate unique key
    int key = ftok("/tmp/ebike-ipc", 999);
    if (key == (key_t)-1) {
        std::printf("ftok failed\n");
        successfulCommunication = false;
        return -1;
    }

    // shmget returns an identifier in shmid
    key_t shmid = shmget(key, 1024, 0666);
    if (shmid == -1) {
        std::printf("shmget failed\n");
        successfulCommunication = false;
        return -1;
    }

    struct shmid_ds info;
    if (shmctl(shmid, IPC_STAT, &info) == -1) {
        std::printf("shmctl failed\n");
        successfulCommunication = false;
        return -1;
    }

    /* Optional: check age or attachments count */
    if (info.shm_nattch == 0) {
        time_t age = time(NULL) - info.shm_ctime;
        if (age > 1 /*second*/) {
            std::printf("shm is old\n");
            successfulCommunication = false;
            return -1;
        }
    }

    // shmat to attach to shared memory
    shm = (SharedMemory*)shmat(shmid, NULL, 0);
    if (shm == (void*)-1) {
        std::printf("shmat failed\n");
        successfulCommunication = false;
        return -1;
    }

    sem_post(&shm->dataClientRead);
    sem_post(&shm->dataServerRead);
    // std::printf("Server: %s\n", shm->dataForServer);
    // std::printf("Client: %s\n", shm->dataForClient);
    // strcpy(shm->dataForClient, "");

    successfulCommunication = true;
    return 0;
};

void IPCClient::stop() {
    sem_post(&shm->dataClientRead);

    // // Detach from shared memory
    // shmdt(shm);

    // // Destroy the shared memory
    // shmctl(shmid, IPC_RMID, NULL);
}

void IPCClient::write(const char *format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    strcpy(shm->dataForServer, buffer);
    sem_post(&shm->dataServerRead);
}

std::string IPCClient::read() {
    sem_wait(&shm->dataClientRead);

    // std::cout << "What was read: " << (std::string)shm->dataForClient << "\n";
    return (std::string)shm->dataForClient;
}