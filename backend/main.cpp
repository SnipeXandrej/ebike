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
#include <format>
#include <signal.h>

#include <wiringPi.h>
#include <mcp23017.h>
#include "ads1256.hpp"

#include "ipcServer.hpp"
#include "utils.hpp"
#include "../comm.h"

#define EBIKE_NAME "EBIKE"
#define EBIKE_VERSION "0.0.0"

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

// BCM pin numbering
#define ADC_DRDY        5
#define ADC_RST         -1
#define ADC_CS          8
#define ADC_PWDN        6
#define ADC_SPI_SPEED   1920000
#define ADC_GAIN        ADS1256_GAIN_16
#define ADC_SAMPLERATE  ADS1256_DRATE_1000SPS
#define ADC_INPUTBUFFER false

#define pinPowerswitch A4_EXP

IPCServer IPC;
toml::table tbl;
ADS1256 ADC;
int spiHandle;

// TODO: do not hardcode filepaths
const char* SETTINGS_FILEPATH = "/home/snipex/.config/ebike/backend.toml";

bool done = false;
std::chrono::duration<double, std::micro> whileLoopUsElapsed;

float acceleration = 0;
float uptimeInSeconds = 0;
float wh_over_km_average = 0;
float motor_rpm = 0;
float speed_kmh = 0;
float throttleLevel = 0;
float throttleBrakeLevel = 0;
bool  powerOn = false;

struct {
    float percentage;
    double watts;
    double current;
    float voltage;
    float voltage_nominal = 72.0;
    float voltage_min = 64.0;
    float voltage_max = 84.0;
    float amphours_min_voltage = 66.0;
    float amphours_max_voltage = 82.0;

    double ampHoursUsed;
    double ampHoursUsedLifetime;
    double ampHoursFullyCharged;
    double ampHoursFullyCharged_tmp;
    double ampHoursRated;

    double wattHoursUsed;
    double wattHoursRated;

    bool charging = false;
} battery;

struct {
    float range;
    float wattHoursUsedOnStart;
    float distance;
    float wattHoursUsed;
    float WhPerKm;
} estimatedRange;

struct {
    double tachometer_abs_previous;
    double tachometer_abs_diff;
    double distance; // in km
    double distanceDiff;
    double wattHoursUsed;
} trip;

struct {
    double trip_distance;    // in km
    double distance;         // in km
    double distance_tmp;
} odometer;

struct {
    bool batteryPercentageVoltageBased = 0;
    bool regenerativeBraking = 0;
} settings;

struct {
    double analog0; // Input 0 // Throttle
    double analog1; // Input 1 // Throtle brake
    double analog2; // Input 2
    double analog3; // Input 3
    double analog4; // Input 4
    double analog5; // Input 5 // Battery voltage
    double analogDiff1; // Input 6+7 // Battery Current
} analogReadings;

void estimatedRangeReset() {
    estimatedRange.wattHoursUsedOnStart  = battery.wattHoursUsed;
    estimatedRange.distance              = 0.0;
    estimatedRange.wattHoursUsed         = 0.0;
}

void tripReset() {
    trip.distance = 0;
    trip.wattHoursUsed = 0;
}

void my_handler(int s) {
    IPC.stop();
    close(spiHandle);
    done = true;
    exit(1);
}

void setupTOML() {
    // TODO: if settings.toml doesnt exist, create it
    tbl = toml::parse_file(SETTINGS_FILEPATH);

    // values
    odometer.distance                       = tbl["odometer"]["distance"].value_or<double>(-1);
    trip.distance                           = tbl["trip"]["distance"].value_or<double>(-1);
    battery.ampHoursUsed                    = tbl["battery"]["ampHoursUsed"].value_or<double>(-1);
    battery.ampHoursFullyCharged            = tbl["battery"]["ampHoursFullyCharged"].value_or<double>(-1);
    battery.ampHoursRated                   = tbl["battery"]["ampHoursRated"].value_or<double>(-1);
    settings.batteryPercentageVoltageBased  = tbl["settings"]["batteryPercentageVoltageBased"].value_or(0);
    settings.regenerativeBraking            = tbl["settings"]["regenerativeBraking"].value_or(0);
}

void saveAll() {
    updateTableValue(SETTINGS_FILEPATH, "odometer", "distance", odometer.distance);
    updateTableValue(SETTINGS_FILEPATH, "trip", "distance", trip.distance);
    updateTableValue(SETTINGS_FILEPATH, "battery", "ampHoursUsed", battery.ampHoursUsed);
    updateTableValue(SETTINGS_FILEPATH, "battery", "ampHoursFullyCharged", battery.ampHoursFullyCharged);
    updateTableValue(SETTINGS_FILEPATH, "battery", "ampHoursRated", battery.ampHoursRated);
    updateTableValue(SETTINGS_FILEPATH, "settings", "batteryPercentageVoltageBased", settings.batteryPercentageVoltageBased);
    updateTableValue(SETTINGS_FILEPATH, "settings", "regenerativeBraking", settings.regenerativeBraking);
}

void setupGPIO() {
    // Initialize
    wiringPiSetupGpio();
    std::printf("wiringPi initialized\n");

    // PWM test
    pinMode(12, PWM_OUTPUT);
    pwmSetClock(1000); // 4.8MHz / divisor
}

void setupMCP() {
    if (!mcp23017Setup(MCP23017_BASEPIN, MCP23017_ADDRESS)) {
        std::printf("MCP23017 failed to initialize, exiting...\n");
        exit(1);
    }

    // Setup pins
    pinMode(        pinPowerswitch, INPUT);
    // pullUpDnControl(pinPowerswitch, PUD_DOWN);
}

void setupADC() {
	ADC.init(&spiHandle, ADC_SPI_SPEED, ADC_DRDY, ADC_RST, ADC_CS, ADC_PWDN, ADC_GAIN, ADC_SAMPLERATE, ADC_INPUTBUFFER);
}

void setupIPC() {
    if (IPC.begin() == -1) {
        std::printf("IPC failed to initialize, exiting...\n");
        exit(1);
    } else {
        std::printf("IPC initialized\n");
    }
}

int main() {
    if (getuid() != 0) {
        std::printf("Run me as root, please :(\n");
        return -1;
    }

    signal(SIGINT, my_handler);

    setupTOML(); // settings
    setupGPIO(); // RPi GPIO
    setupMCP();  // Pin Expander
    setupADC();  // ADC
    setupIPC();  // IPC

    std::thread readThread([&] {
        std::printf("Started IPC Read\n");
        while(!done) {
            std::string toSend;
            std::string whatWasRead;
            whatWasRead = IPC.read();
            // std::cout << whatWasRead << "\n";

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
                        commAddValue(&toSend, battery.ampHoursUsedLifetime, 2);
                        commAddValue(&toSend, battery.ampHoursFullyCharged, 2);
                        commAddValue(&toSend, battery.ampHoursRated, 2);
                        commAddValue(&toSend, battery.percentage, 1);
                        commAddValue(&toSend, battery.voltage_min, 1);
                        commAddValue(&toSend, battery.voltage_max, 1);
                        commAddValue(&toSend, battery.voltage_nominal, 1);
                        commAddValue(&toSend, battery.amphours_min_voltage, 1);
                        commAddValue(&toSend, battery.amphours_max_voltage, 1);
                        commAddValue(&toSend, battery.charging, 0);

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::ARE_YOU_ALIVE:
                        commAddValue(&toSend, COMMAND_ID::ARE_YOU_ALIVE, 0);

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::GET_STATS:
                        commAddValue(&toSend, COMMAND_ID::GET_STATS, 0);
                        commAddValue(&toSend, speed_kmh, 1);
                        commAddValue(&toSend, motor_rpm, 0);
                        commAddValue(&toSend, odometer.distance, 1);
                        commAddValue(&toSend, trip.distance, 2);
                        commAddValue(&toSend, 0, 0); // was gearCurrent.level
                        commAddValue(&toSend, 0, 0); // was gearCurrent.maxCurrent
                        commAddValue(&toSend, 0, 0); // was selectedPowerMode
                        commAddValue(&toSend, 0, 1); // VESC.data.avgMotorCurrent
                        commAddValue(&toSend, wh_over_km_average, 1);
                        commAddValue(&toSend, estimatedRange.WhPerKm, 1);
                        commAddValue(&toSend, estimatedRange.range, 1);
                        commAddValue(&toSend, 10, 1); // VESC.data.tempMotor
                        commAddValue(&toSend, uptimeInSeconds, 0);
                        commAddValue(&toSend, whileLoopUsElapsed.count(), 0);
                        commAddValue(&toSend, 1100, 0); // timeCore1
                        commAddValue(&toSend, acceleration, 1);
                        commAddValue(&toSend, powerOn, 0); //POWER_ON
                        commAddValue(&toSend, settings.regenerativeBraking, 0);

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::SET_ODOMETER:
                        odometer.distance = (float)getValueFromPacket(packet, 1);
                        toSend.append(std::format("{};Odometer was set to: {} km;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG), odometer.distance));
                        break;

                    case COMMAND_ID::SAVE_PREFERENCES:
                        saveAll();
                        toSend.append(std::format("{};Preferences were manually saved;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                        break;

                    case COMMAND_ID::RESET_TRIP:
                        tripReset();
                        toSend.append(std::format("{};Trip was reset;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                        break;

                    case COMMAND_ID::RESET_ESTIMATED_RANGE:
                        estimatedRangeReset();
                        toSend.append(std::format("{};Estimated range was reset;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                        break;

                    case COMMAND_ID::GET_FW:
                        commAddValue(&toSend, COMMAND_ID::GET_FW, 0);
                        toSend.append(std::format("{};{}; {} {};", EBIKE_NAME, EBIKE_VERSION, __DATE__, __TIME__)); // NAME, VERSION, COMPILE DATE/TIME

                        toSend.append("\n");
                        break;

                    // case COMMAND_ID::PING:
                    //     display.ping();
                    //     break;

                    // case COMMAND_ID::TOGGLE_FRONT_LIGHT:

                    //     break;

                    case COMMAND_ID::SET_AMPHOURS_USED_LIFETIME:
                        battery.ampHoursUsedLifetime = (float)getValueFromPacket(packet, 1);
                        toSend.append(std::format("{};Amphours used (Lifetime) was set to: {} Ah;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG), battery.ampHoursUsedLifetime));
                        break;

                    // case COMMAND_ID::GET_VESC_MCCONF:
                    //     idx = FNPOOL::GET_MCCONF_TEMP;
                    //     if (xQueueSend(core0_queue, &idx, portMAX_DELAY) == pdTRUE) {
                    //         toSend.append(std::format("{};xQueueSent!;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                    //     };

                    //     break;

                    // case COMMAND_ID::SET_VESC_MCCONF:
                    //     VESC.data_mcconf.l_current_min_scale = (float)getValueFromPacket(packet, 1);
                    //     VESC.data_mcconf.l_current_max_scale = (float)getValueFromPacket(packet, 2);
                    //     VESC.data_mcconf.l_min_erpm = (float)getValueFromPacket(packet, 3) / 1000.00;
                    //     VESC.data_mcconf.l_max_erpm = (float)getValueFromPacket(packet, 4) / 1000.00;
                    //     VESC.data_mcconf.l_min_duty = (float)getValueFromPacket(packet, 5);
                    //     VESC.data_mcconf.l_max_duty = (float)getValueFromPacket(packet, 6);
                    //     VESC.data_mcconf.l_watt_min = (float)getValueFromPacket(packet, 7);
                    //     VESC.data_mcconf.l_watt_max = (float)getValueFromPacket(packet, 8);
                    //     VESC.data_mcconf.l_in_current_min = (float)getValueFromPacket(packet, 9);
                    //     VESC.data_mcconf.l_in_current_max = (float)getValueFromPacket(packet, 10);
                    //     VESC.data_mcconf.name = getValueFromPacket_string(packet, 11);
                    //     VESC.setMcconfTempValues();

                    //     toSend.append(std::format("{};VESC McConf was sent;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                    //     break;

                    case COMMAND_ID::SET_AMPHOURS_CHARGED:
                        {
                            float newValue = getValueFromPacket(packet, 1);

                            battery.ampHoursFullyCharged = newValue;
                            battery.ampHoursFullyCharged_tmp = newValue;

                            toSend.append(std::format("{};Amphours charged was set to: {} Ah;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG), newValue));
                        }
                        break;

                    case COMMAND_ID::TOGGLE_CHARGING_STATE:
                        battery.charging = !battery.charging;

                        toSend.append(std::format("{};Charging state was toggled, now set to: {};\n", static_cast<int>(COMMAND_ID::BACKEND_LOG), battery.charging));
                        break;

                    case COMMAND_ID::TOGGLE_REGEN_BRAKING:
                        settings.regenerativeBraking = !settings.regenerativeBraking;

                        toSend.append(std::format("{};Regenerative braking state was toggled, now set to: {};\n", static_cast<int>(COMMAND_ID::BACKEND_LOG), settings.regenerativeBraking));
                        break;
                    case COMMAND_ID::GET_ANALOG_READINGS:
                        commAddValue(&toSend, COMMAND_ID::GET_ANALOG_READINGS, 0);
                        commAddValue(&toSend, analogReadings.analog0, 15);
                        commAddValue(&toSend, analogReadings.analog1, 15);
                        commAddValue(&toSend, analogReadings.analog2, 15);
                        commAddValue(&toSend, analogReadings.analog3, 15);
                        commAddValue(&toSend, analogReadings.analog4, 15);
                        commAddValue(&toSend, analogReadings.analog5, 15);
                        commAddValue(&toSend, analogReadings.analogDiff1, 15);

                        toSend.append("\n");
                        break;
                }
            }
            IPC.write(toSend.c_str());
        }
    });

    std::thread uptimeThread([&] {
        std::printf("Started uptime counting\n");
        static std::chrono::duration<double, std::micro> usElapsed;
        while (!done) {
            auto t1 = std::chrono::high_resolution_clock::now();
            uptimeInSeconds += usElapsed.count() / 1000000.0;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            usElapsed = std::chrono::high_resolution_clock::now() - t1;
        }
    });

    double data;
    std::printf("Enter loop\n");
    while (!done) {
		auto t1 = std::chrono::high_resolution_clock::now();

        bool powerOn_tmp = digitalRead(pinPowerswitch);
        if (powerOn_tmp != powerOn) {
            powerOn = powerOn_tmp;
            if (powerOn) {
                // run code when turned on
                std::printf("Power on\n");

            }

            if (!powerOn) {
                // run code when turned off
                std::printf("Power off\n");

            }
        }

        // PWM
        pwmWrite(12, 256); // 0 - 1023

        // Analog reading
		data = ADC.readChannel(0);
		analogReadings.analogDiff1 = ADC.convertToVoltage(data) - 0.000002;
		data = ADC.readChannel(1);
		analogReadings.analog0 = ADC.convertToVoltage(data) - 0.0094;
		data = ADC.readChannel(2);
		analogReadings.analog1 = ADC.convertToVoltage(data) - 0.0094;
		data = ADC.readChannel(3);
		analogReadings.analog2 = ADC.convertToVoltage(data) - 0.00945;
        data = ADC.readChannel(4);
		analogReadings.analog3 = ADC.convertToVoltage(data) - 0.00945;
        data = ADC.readChannel(5);
		analogReadings.analog4 = ADC.convertToVoltage(data) - 0.00945;
		data = ADC.readDiffChannel(3);
		analogReadings.analog5 = ADC.convertToVoltage(data) - 0.00941; // OK

        // throttleLevel
        static float throttleR1 = 10000;
        static float throttleR2 = 560;
        static float throttleVMaxInput = 5;
        static float throttleVMax = (throttleVMaxInput * throttleR2) / (throttleR1 + throttleR2);

        // throttleBrakeLevel
        static float throttleBrakeR1 = 10000;
        static float throttleBrakeR2 = 560;
        static float throttleBrakeVMaxInput = 5;
        static float throttleBrakeVMax = (throttleBrakeVMaxInput * throttleBrakeR2) / (throttleBrakeR1 + throttleBrakeR2);

        // battery.voltage
        static float batteryVoltageR1 = 220000;
        static float batteryVoltageR2 = 560;
        static float batteryVoltageVMaxInput = 100;
        static float batteryVoltageVMax = (batteryVoltageVMaxInput * batteryVoltageR2) / (batteryVoltageR1 + batteryVoltageR2);

        static double inputReadMaxV = (5.0 / 16.0);
        static double mvPerAmp = 0.0015;

        // map those readings
        throttleLevel       = map_f_nochecks(analogReadings.analog0, 0.0, throttleVMax, 0.0, 100);
        throttleBrakeLevel  = map_f_nochecks(analogReadings.analog1, 0.0, throttleBrakeVMax, 0.0, 100);
        battery.voltage     = map_f_nochecks(analogReadings.analog5, 0.0, batteryVoltageVMax, 0.0, batteryVoltageVMaxInput);
        battery.current     = map_d_nochecks(analogReadings.analogDiff1, 0.0, inputReadMaxV, 0.0, (inputReadMaxV/mvPerAmp));

        // calculate other stuff
        battery.watts = battery.voltage * battery.current;

        whileLoopUsElapsed = std::chrono::high_resolution_clock::now() - t1;
    }

    uptimeThread.join();
    readThread.join();

    return 0;
}
