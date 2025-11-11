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

#include "myUart.hpp"
#include "VescUart/VescUart.h"
#include "ipcServer.hpp"
#include "utils.hpp"
#include "../comm.h"
#include "inputOffset.h"
#include "map.hpp"

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
VescUart VESC;
MyUart uartVESC;
int spiHandle;
ThrottleMap throttle;

InputOffset PotThrottleCorrection;
MovingAverage BatVoltageMovingAverage;
MovingAverage PotThrottleMovingAverage;
MovingAverage BatWattMovingAverage;
MovingAverage WhOverKmMovingAverage;
MovingAverage batteryAuxWattsMovingAverage;
MovingAverage batteryAuxMovingAverage;
MovingAverage batteryWattsMovingAverage;
MovingAverage wattageMoreSmooth_MovingAverage;
struct {
    MovingAverage brakingCurrent;
} movingAverages;

// TODO: do not hardcode filepaths
const char* SETTINGS_FILEPATH = "/home/snipex/.config/ebike/backend.toml";

bool done = false;
std::chrono::duration<double, std::micro> whileLoopUsElapsed;

float acceleration = 0;
float uptimeInSeconds = 0;
float wh_over_km_average = 0;
float wh_over_km = 0;
float motor_rpm = 0;
float speed_kmh = 0;
float throttleLevel = 0;
float throttleBrakeLevel = 0;
bool  powerOn = false;
bool  BRAKING = false;

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
    double ampHoursRated_tmp;

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

struct {
    int poles = 18;
    int magnetPairs = 3;
    float rpmPerKmh = 90.04;
} motor;

struct {
    float diameter = 63.0;
    float gear_ratio = 10.6875;
} wheel;

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

float clampValue(float input, float clampTo) {
    float output = 0.0;

    if (input < clampTo) {
        output = input;
    }

    if (input >= clampTo) {
        output = clampTo;
    }

    return output;
}

float maxCurrentAtRPM(float rpm) {
    float RPM1 = 360.0;
    float RPM2 = 450.0;
    float currentBeforeRPM1 = 150.0;

    if (rpm <= RPM1) {
        return currentBeforeRPM1;
    }

    if (rpm <= RPM2) {
        // Linear interpolation between 120A at 430 ERPM and 190A at 1000 ERPM
        float slope = (VESC.data.maxMotorCurrent - currentBeforeRPM1) / (RPM2 - RPM1);
        return currentBeforeRPM1 + slope * (rpm - RPM1);
    }

    if (rpm > RPM2) {
        return VESC.data.maxMotorCurrent;
    }

    return 0.0;
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
    std::printf("[wiringPi] initialized\n");

    // PWM test
    pinMode(12, PWM_OUTPUT);
    pwmSetClock(1000); // 4.8MHz / divisor
}

void setupMCP() {
    if (!mcp23017Setup(MCP23017_BASEPIN, MCP23017_ADDRESS)) {
        std::printf("[MCP23017] failed to initialize, exiting...\n");
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
        std::printf("[IPC] failed to initialize, exiting...\n");
        exit(1);
    } else {
        std::printf("[IPC] initialized\n");
    }
}

int main() {
    if (getuid() != 0) {
        std::printf("Run me as root, please :(\n");
        return -1;
    }
    signal(SIGINT, my_handler);

    VESC.data.maxMotorCurrent = 250.0; // TODO: retrieve it from VESC
    std::vector<Point> customThrottleCurve = {
        {0, 0},
        {8, 8},
        {15, 13},
        {20, 18},
        {30, 30},
        {40, 40},
        {50, 60},
        {75, 150},
        {87, 200},
        {100, 250}
    };
    throttle.setCurve(customThrottleCurve);

    setupIPC();  // IPC
    setupTOML(); // settings
    setupGPIO(); // RPi GPIO
    setupMCP();  // Pin Expander
    setupADC();  // ADC
    uartVESC.begin(B115200);
    VESC.setSerialPort(&uartVESC);

    std::thread readThread([&] {
        std::printf("[readThread] Started IPC Read\n");
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
                        commAddValue(&toSend, VESC.data.avgMotorCurrent, 1);
                        commAddValue(&toSend, wh_over_km_average, 1);
                        commAddValue(&toSend, estimatedRange.WhPerKm, 1);
                        commAddValue(&toSend, estimatedRange.range, 1);
                        commAddValue(&toSend, VESC.data.tempMotor, 1);
                        commAddValue(&toSend, uptimeInSeconds, 0);
                        commAddValue(&toSend, whileLoopUsElapsed.count(), 0);
                        commAddValue(&toSend, 1100, 0); // timeCore1
                        commAddValue(&toSend, acceleration, 1);
                        commAddValue(&toSend, powerOn, 0);
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

                    case COMMAND_ID::GET_VESC_MCCONF:
                        if (VESC.getMcconfTempValues()) {
                            commAddValue(&toSend, COMMAND_ID::GET_VESC_MCCONF, 0);
                            commAddValue(&toSend, VESC.data_mcconf.l_current_min_scale, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_current_max_scale, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_min_erpm, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_max_erpm, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_min_duty, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_max_duty, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_watt_min, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_watt_max, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_in_current_min, 4);
                            commAddValue(&toSend, VESC.data_mcconf.l_in_current_max, 4);
                            toSend.append(VESC.data_mcconf.name); toSend.append(";");
                            toSend.append("\n");

                            toSend.append(std::format("{};Latest McConf values retrieved!;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                        } else {
                            toSend.append(std::format("{};Latest McConf values did NOT get retrieved!;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                        }
                        break;

                    case COMMAND_ID::SET_VESC_MCCONF:
                        VESC.data_mcconf.l_current_min_scale = (float)getValueFromPacket(packet, 1);
                        VESC.data_mcconf.l_current_max_scale = (float)getValueFromPacket(packet, 2);
                        VESC.data_mcconf.l_min_erpm = (float)getValueFromPacket(packet, 3) / 1000.00;
                        VESC.data_mcconf.l_max_erpm = (float)getValueFromPacket(packet, 4) / 1000.00;
                        VESC.data_mcconf.l_min_duty = (float)getValueFromPacket(packet, 5);
                        VESC.data_mcconf.l_max_duty = (float)getValueFromPacket(packet, 6);
                        VESC.data_mcconf.l_watt_min = (float)getValueFromPacket(packet, 7);
                        VESC.data_mcconf.l_watt_max = (float)getValueFromPacket(packet, 8);
                        VESC.data_mcconf.l_in_current_min = (float)getValueFromPacket(packet, 9);
                        VESC.data_mcconf.l_in_current_max = (float)getValueFromPacket(packet, 10);
                        VESC.data_mcconf.name = getValueFromPacket_string(packet, 11);
                        VESC.setMcconfTempValues();

                        toSend.append(std::format("{};VESC McConf was sent;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                        break;

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
        std::printf("[uptimeThread] Started uptime counting\n");
        static std::chrono::duration<double, std::micro> usElapsed;
        while (!done) {
            auto t1 = std::chrono::high_resolution_clock::now();
            uptimeInSeconds += usElapsed.count() / 1000000.0;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            usElapsed = std::chrono::high_resolution_clock::now() - t1;
        }
    });

    std::thread VESCThread([&] {
        while (!done) {
            if (VESC.getVescValues()) {
                // if the previous measurement is bigger than the current, that means
                // that the VESC was probably powered off and on, so the stats got reset...
                // So this makes sure that we do not make a tachometer_abs_diff thats suddenly a REALLY
                // large number and therefore screw up our distance measurement
                if (trip.tachometer_abs_previous > VESC.data.tachometerAbs) {
                    trip.tachometer_abs_previous = VESC.data.tachometerAbs;
                }

                // prevent the diff to be something extremely big
                if ((VESC.data.tachometerAbs - trip.tachometer_abs_previous) >= 1000) {
                    trip.tachometer_abs_previous = VESC.data.tachometerAbs;
                }

                if (trip.tachometer_abs_previous < VESC.data.tachometerAbs) {
                    trip.tachometer_abs_diff = VESC.data.tachometerAbs - trip.tachometer_abs_previous;
                    // Serial.printf("Trip tacho diff: %f\n", trip.tachometer_abs_diff); // ~90 for 80km/h

                    trip.distanceDiff = ((trip.tachometer_abs_diff / (double)motor.poles) / (double)wheel.gear_ratio) * (double)wheel.diameter * 3.14159265 / 100000.0; // divide by 100000 for trip distance to be in kilometers

                    // TODO: reset the trip after pressing a button? reset the trip after certain amount of time has passed?
                    trip.distance += trip.distanceDiff;
                    odometer.distance += trip.distanceDiff;
                    estimatedRange.distance += trip.distanceDiff;

                    trip.tachometer_abs_previous = VESC.data.tachometerAbs;
                }

                motor_rpm = (VESC.data.rpm / (float)motor.magnetPairs);
                speed_kmh = (motor_rpm / wheel.gear_ratio) * wheel.diameter * 3.14159265f * 60.0f/*minutes*/ / 100000.0f/*1 km in cm*/;
            }
        }
    });

    double data;
    std::printf("[Main] Enter loop\n");
    while (!done) {
		auto t1 = std::chrono::high_resolution_clock::now();

        bool powerOn_tmp = digitalRead(pinPowerswitch);
        if (powerOn_tmp != powerOn) {
            powerOn = powerOn_tmp;
            if (powerOn) {
                // run code when turned on
                std::printf("[Main] Power on\n");

            }

            if (!powerOn) {
                // run code when turned off
                std::printf("[Main] Power off\n");

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
        throttleLevel       = 7; //map_f_nochecks(analogReadings.analog0, 0.0, throttleVMax, 0.0, 100);
        throttleBrakeLevel  = map_f_nochecks(analogReadings.analog1, 0.0, throttleBrakeVMax, 0.0, 100);
        battery.voltage     = map_f_nochecks(analogReadings.analog5, 0.0, batteryVoltageVMax, 0.0, batteryVoltageVMaxInput);
        battery.current     = map_d_nochecks(analogReadings.analogDiff1, 0.0, inputReadMaxV, 0.0, (inputReadMaxV/mvPerAmp));

        // calculate other stuff
        battery.watts = battery.voltage * battery.current;
        battery.wattHoursRated = battery.voltage_nominal* battery.ampHoursRated;

        // BATTERY VOLTAGE
        BatVoltageMovingAverage.initInput(battery.voltage);

        // BATTERY CURRENT
        // getBatteryCurrent sets the battery.current var to a new value
        // Calculates wattHoursUsed, ampHoursUsed, ampHoursUsedLifetime, ...
        static float _batteryCurrentUsedInElapsedTime, _batteryWattHoursUsedUsedInElapsedTime;

        _batteryCurrentUsedInElapsedTime = battery.current / (1000000 / whileLoopUsElapsed.count());
        battery.ampHoursUsed            += _batteryCurrentUsedInElapsedTime / 3600.0;
        if (_batteryCurrentUsedInElapsedTime >= 0.0) {
            battery.ampHoursUsedLifetime    += _batteryCurrentUsedInElapsedTime / 3600.0;
        }

        _batteryWattHoursUsedUsedInElapsedTime = (battery.current * battery.voltage) / (1000000 / whileLoopUsElapsed.count());
        battery.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;
        if ((_batteryWattHoursUsedUsedInElapsedTime >= 0.0 || BRAKING) && !battery.charging) {
            trip.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;
            estimatedRange.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;
        }

        // BRAKING = digitalRead(pinBrake);
        // TODO: feature idea: when we manually start to roll the bike from a standstill, dont activate braking until some throttle is applied
        if (speed_kmh > 1.0 && throttleLevel == 0) {
            BRAKING = true;
        } else {
            BRAKING = false;
        }

        static auto timeAmphoursMinVoltage = std::chrono::high_resolution_clock::now();
        static auto timeWaitBeforeChangingToCharging = std::chrono::high_resolution_clock::now();
        static std::chrono::duration<double, std::milli> timeAmphoursMinVoltageMsElapsed;
        static std::chrono::duration<double, std::milli> timeWaitBeforeChangingToChargingMsElapsed;

        // automatically change the battery.charging status to true if its false & we aint braking & the current is less than 0 (meaning it's charging)
        // we don't have dedicated sensing pin for the charger being connected, yet...
        // after that wait one second, just to make sure it's not a fluke, so it doesnt just randomly put it in charging mode because of a small anomaly...
        if (!battery.charging && !BRAKING && battery.current < 0.0) {
            timeWaitBeforeChangingToChargingMsElapsed = std::chrono::high_resolution_clock::now() - timeWaitBeforeChangingToCharging;
            if (timeWaitBeforeChangingToChargingMsElapsed.count() >= 1000) {
                battery.charging = true;
            }
        } else {
            timeWaitBeforeChangingToCharging = std::chrono::high_resolution_clock::now();
        }

        // Battery charge tracking stuff
        if (settings.batteryPercentageVoltageBased) {
            battery.percentage = map_f(battery.voltage, battery.voltage_min, battery.voltage_max, 0, 100);
        } else {
            battery.percentage = map_f_nochecks(battery.ampHoursUsed, 0.0, battery.ampHoursRated, 100.0, 0.0);
        }

        BatVoltageMovingAverage.moveAverage(battery.voltage);
        if (!battery.charging && (BatVoltageMovingAverage.output <= battery.amphours_min_voltage) && (battery.current <= 3.0 && battery.current >= 0.0)) {
            timeAmphoursMinVoltageMsElapsed = std::chrono::high_resolution_clock::now() - timeAmphoursMinVoltage;
            if (timeAmphoursMinVoltageMsElapsed.count() >= 5000) {
                battery.ampHoursRated_tmp = battery.ampHoursUsed; // save ampHourUsed to ampHourRated_tmp...
                                                                // this temporary value will later get applied to the actual
                                                                // ampHourRated variable when the battery is done charging so
                                                                // the estimated range and other stuff doesn't suddenly get screwed
            }
        } else {
            timeAmphoursMinVoltage = std::chrono::high_resolution_clock::now();
        }

        if (BatVoltageMovingAverage.output >= battery.amphours_max_voltage) {
            // TODO: use dedicated current sensing for charging
            if (battery.charging && (battery.current <= 0.0 && battery.current >= -0.5)) {
                battery.ampHoursUsed = 0;
                battery.wattHoursUsed = 0;
                if (battery.ampHoursRated_tmp != 0.0) {
                    battery.ampHoursRated = battery.ampHoursRated_tmp;
                }
            }
        }

        // Throttle
        if (!powerOn || battery.charging) {
            VESC.setCurrent(0.0f);
        } else if (BRAKING && settings.regenerativeBraking) {
            static float brakingCurrentMax = 100.0;

            // gradually increase braking current
            VESC.setBrakeCurrent(movingAverages.brakingCurrent.moveAverage(brakingCurrentMax));
        } else {
            float vescCurrent = clampValue(
                                            throttle.map(throttleLevel),
                                            maxCurrentAtRPM(VESC.data.rpm / motor.magnetPairs)
                                        );
            VESC.setCurrent(vescCurrent);

            // reset regen braking
            movingAverages.brakingCurrent.setInput(0.0f);
        }

        // wh_over_km
        float wh_over_km_tmp = battery.watts / speed_kmh;
        if (wh_over_km_tmp > 199.9) {
            wh_over_km = 199.9;
        } else if (wh_over_km_tmp < 0.0) {
            wh_over_km = 199.9;
        } else if (wh_over_km_tmp != wh_over_km_tmp) {
            wh_over_km = 199.9;
        } else {
            wh_over_km = battery.watts / speed_kmh;
        }

        // wh_over_km_average
        double tmp = trip.wattHoursUsed / trip.distance;
        tmp != tmp ? wh_over_km_average = -1 : wh_over_km_average = tmp;

        // estimatedRange.range
        estimatedRange.WhPerKm = estimatedRange.wattHoursUsed / estimatedRange.distance;
        tmp = (battery.wattHoursRated - battery.wattHoursUsed) / estimatedRange.WhPerKm;
        tmp != tmp ? estimatedRange.range = -1 : estimatedRange.range = tmp;

        whileLoopUsElapsed = std::chrono::high_resolution_clock::now() - t1;
    }

    uptimeThread.join();
    readThread.join();
    VESCThread.join();

    return 0;
}
