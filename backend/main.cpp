// high_resolution_clock::now has an overhead of 0.161812 us

// MCP23017 at 1.2MHz I2C
// switches between HIGH and LOW at 6.85kHz
// digitalRead takes around 88.5us
// digitalWrite takes around 73 us

#include <cstdio>
#include <thread>
#include <chrono>
#include <iostream>
#include <signal.h>

#include <wiringPi.h>
#include <mcp23017.h>

#include "ipcServer.hpp"
IPCServer IPC;

#define MCP23017_ADDRESS 0x20
#define MCP23017_BASEPIN 100
#define A0_EXP MCP23017_BASEPIN+0
#define A1_EXP MCP23017_BASEPIN+1
#define A2_EXP MCP23017_BASEPIN+2
#define A3_EXP MCP23017_BASEPIN+3
#define A4_EXP MCP23017_BASEPIN+4
#define A5_EXP MCP23017_BASEPIN+5
#define A6_EXP MCP23017_BASEPIN+6
#define A7_EXP MCP23017_BASEPIN+7
#define B0_EXP MCP23017_BASEPIN+8
#define B1_EXP MCP23017_BASEPIN+9
#define B2_EXP MCP23017_BASEPIN+10
#define B3_EXP MCP23017_BASEPIN+11
#define B4_EXP MCP23017_BASEPIN+12
#define B5_EXP MCP23017_BASEPIN+13
#define B6_EXP MCP23017_BASEPIN+14
#define B7_EXP MCP23017_BASEPIN+15

void my_handler(int s) {
    // IPC.stop();
    exit(1);
}

void setupGPIO() {
    // Initialize
    wiringPiSetupGpio();
    mcp23017Setup(MCP23017_BASEPIN, MCP23017_ADDRESS);

    // Setup pins
    pinMode(        A0_EXP, OUTPUT);
    pullUpDnControl(A0_EXP, PUD_DOWN);
}

int main() {
    signal(SIGINT, my_handler);

    setupGPIO();
    IPC.begin();

    std::thread readThread([&] {
        while(1) {
            std::cout << IPC.read() << "\n";
        }
    });

    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        digitalWrite(A0_EXP, HIGH);
        IPC.write("Pin is: %d\n", digitalRead(A0_EXP));

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        digitalWrite(A0_EXP, LOW);
        IPC.write("Pin is: %d\n", digitalRead(A0_EXP));
    }

    return 0;
}
