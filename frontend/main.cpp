#include "ipcClient.hpp"
#include <signal.h>

IPCClient IPC;


void my_handler(int s) {
    // IPC.stop();
    exit(1);
}

int main() {
    signal(SIGINT, my_handler);
    IPC.begin();

    std::thread inputThread([&] {
        while(1) {
            std::string input;
            std::cin >> input;

            IPC.write(input.c_str());
        }
    });

    while(1) {
        std::cout << IPC.read();
    }

return 0;
}