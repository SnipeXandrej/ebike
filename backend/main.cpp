// high_resolution_clock::now has an overhead of 0.161812 us

// MCP23017 at 1.2MHz I2C
// switches between HIGH and LOW at 6.85kHz
// digitalRead takes around 88.5us
// digitalWrite takes around 73 us

#include <cstdio>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <signal.h>

#include <wiringPi.h>
#include <mcp23017.h>

#include "ipcServer.hpp"
#include "utils.hpp"
#include "../comm.h"
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

struct {
    float voltage;
    float voltage_min;
    float voltage_max;
    float current;
    float ampHoursUsed;
    float ampHoursUsedLifetime;
    float watts;
    float wattHoursUsed;
    float percentage;
    float ampHoursRated;
    float ampHoursRated_tmp;
    float amphours_min_voltage;
    float amphours_max_voltage;
    bool charging = false;

    float wattHoursRated;
    float nominalVoltage;
} battery;

void my_handler(int s) {
    // IPC.stop();
    exit(1);
}

void setupGPIO() {
    // Initialize
    wiringPiSetupGpio();
    std::printf("wiringPi initialized\n");

    if (!mcp23017Setup(MCP23017_BASEPIN, MCP23017_ADDRESS)) {
        std::printf("MCP23017 failed to initialize, exiting...\n");
        exit(1);
    }

    // Setup pins
    pinMode(        A0_EXP, OUTPUT);
    pullUpDnControl(A0_EXP, PUD_DOWN);
}

int main() {
    signal(SIGINT, my_handler);

    setupGPIO();
    if (IPC.begin() == -1) {
        std::printf("IPC failed to initialize, exiting...\n");
        exit(1);
    } else {
        std::printf("IPC initialized successfully\n");
    }

    std::thread readThread([&] {
        std::printf("Started IPC Read\n");
        while(1) {
            std::string toSend;
            std::string whatWasRead;
            whatWasRead = IPC.read();

            auto readStringPacket = split(whatWasRead, '\n');

            for (int i = 0; i < (int)readStringPacket.size(); i++) {
                auto packet = split(readStringPacket[i], ';');

                bool isItStoiSafe = true;
                for (char c : packet[0]) {
                    if (!isdigit(c)) isItStoiSafe = false;
                }

                int packet_command_id = -1;
                if (isItStoiSafe)
                    packet_command_id = std::stoi(packet[0]);

                switch(packet_command_id) {
                    case COMMAND_ID::GET_BATTERY:
                        commAddValue(&toSend, COMMAND_ID::GET_BATTERY, 0);
                        commAddValue(&toSend, battery.voltage, 2);
                        commAddValue(&toSend, battery.current, 4);
                        commAddValue(&toSend, battery.watts, 1);
                        commAddValue(&toSend, battery.wattHoursUsed, 1);
                        commAddValue(&toSend, battery.ampHoursUsed, 6);
                        commAddValue(&toSend, battery.ampHoursUsedLifetime, 1);
                        commAddValue(&toSend, battery.ampHoursRated, 2);
                        commAddValue(&toSend, battery.percentage, 1);
                        commAddValue(&toSend, battery.voltage_min, 1);
                        commAddValue(&toSend, battery.voltage_max, 1);
                        commAddValue(&toSend, battery.nominalVoltage, 1);
                        commAddValue(&toSend, battery.amphours_min_voltage, 1);
                        commAddValue(&toSend, battery.amphours_max_voltage, 1);
                        commAddValue(&toSend, battery.charging, 0);

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::ARE_YOU_ALIVE:
                        commAddValue(&toSend, COMMAND_ID::ARE_YOU_ALIVE, 0);

                        toSend.append("\n");
                        break;
                }
                IPC.write(toSend.c_str());
            }
        }
    });

    std::printf("Entering loop\n");
    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        digitalWrite(A0_EXP, HIGH);

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        digitalWrite(A0_EXP, LOW);
    }

    return 0;
}
