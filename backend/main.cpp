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
#include <wiringPiI2C.h>
#include <mcp23017.h>
#include <mcp3004.h>

#include "ads1115.hpp"
#include "myUart.hpp"
#include "VescUart/VescUart.h"
#include "server.hpp"
#include "utils.hpp"
#include "../comm.h"
#include "inputOffset.h"
#include "map.hpp"
#include "profiles.hpp"
#include "../timer.hpp"
#include "../valueTransition.hpp"

#define EBIKE_NAME "EBIKE"
#define EBIKE_VERSION "0.0.0"

// MCP23017
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

//MCP3008
#define MCP3008_SPICHAN 0
#define MCP3008_BASEPIN 200
#define A0_ADC MCP3008_BASEPIN+0
#define A1_ADC MCP3008_BASEPIN+1
#define A2_ADC MCP3008_BASEPIN+2
#define A3_ADC MCP3008_BASEPIN+3
#define A4_ADC MCP3008_BASEPIN+4
#define A5_ADC MCP3008_BASEPIN+5
#define A6_ADC MCP3008_BASEPIN+6
#define A7_ADC MCP3008_BASEPIN+7

// ADS1115
#define ADS1115_ADDRESS 0x4a
#define ADS1115_BASEPIN 300

// Pins
#define pinPowerswitch  A0_EXP
#define pinPWM_fan      12

ServerSocket IPC;
toml::table tbl;
VescUart    VESC;
MyUart      uartVESC;
ThrottleMap throttleMap;
ThrottleMap brakeMap;
PowerProfiles PP;

struct {
    MovingAverage potThrottle;
    MovingAverage brakeThrottle;
    MovingAverage brakingCurrent;
    MovingAverage batteryCurrentForFrontend;
} movingAverages;

struct {
    std::thread uptimeCounter;
    std::thread throttle;
    std::thread vescValueProcessing;
    std::thread IPCRead;
} threads;

struct {
    Timer throttleCreep;
} timer;

struct {
    ValueTransition throttleShockCurrent;
    ValueTransition throttleReal;
    ValueTransition throttleToBrake;
    ValueTransition toRealBrake;
} valueTransition;

// TODO: do not hardcode filepaths
const char* SETTINGS_FILEPATH = "/home/snipex/.config/ebike/backend.toml";
std::chrono::duration<double, std::micro> whileLoopUsElapsed;
float acceleration = 0;
float uptimeInSeconds = 0;
float motor_rpm = 0;
float speed_kmh = 0;
float throttleLevel = 0;
float brakeLevel = 0;
bool  powerOn = false;
bool  done = false;
float maxBrakingCurrent = 0.0;
std::string toSendExtra;

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

    double currentForFrontend;

    double ampHoursUsed;
    double ampHoursUsedLifetime;
    double ampHoursFullyCharged;
    double ampHoursFullyCharged_tmp;
    double ampHoursFullyChargedWhenNew;

    double wattHoursUsed;
    double wattHoursFullyDischarged;
    double wattHoursFullyDischarged_tmp;

    bool charging = false;
} battery;

struct {
    float range;
    float wattHoursUsedOnStart;
    float distance;
    float wattHoursUsed;
    float WhPerKm;
} estimatedRange;

struct Trip {
    double distance; // in km
    double wattHoursUsed;
    double wattHoursConsumed;
    double wattHoursRegenerated;
};
Trip trip_A, trip_B;

struct {
    double trip_distance;    // in km
    double distance;         // in km
} odometer;

struct {
    bool batteryPercentageVoltageBased = 0;
    bool regenerativeBraking = 0;
    bool minimizeDrivetrainBacklash = 0;
} settings;

struct {
    double analog0; // Throttle
    double analog1; // Throtle brake
    double analog2;
    double analog3;
    double analog4;
    double analog5;
    double analog6;
    double analog7; // Battery voltage
} analogReadings;

struct {
    int poles = 18;
    int magnetPairs = 3;
    float rpmPerKmh = 0;
} motor;

struct {
    float diameter = 63.0;
    float gear_ratio = 12.1125; // (34/10)*(57/16)
    float rpmPerKmh = 0;
} wheel;

void estimatedRangeReset() {
    estimatedRange.wattHoursUsedOnStart  = battery.wattHoursUsed;
    estimatedRange.distance              = 0.0;
    estimatedRange.wattHoursUsed         = 0.0;
}

void tripReset(Trip *trip) {
    trip->distance = 0;
    trip->wattHoursConsumed = 0;
    trip->wattHoursRegenerated = 0;
}

void my_handler(int s) {
    (void)(s); // suppress unused parameter compiler warning

    IPC.stop();
    done = true;
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

void saveAll() {
    updateTableValue(SETTINGS_FILEPATH, "odometer", "distance", odometer.distance);
    updateTableValue(SETTINGS_FILEPATH, "trip", "distance", trip_A.distance);
    updateTableValue(SETTINGS_FILEPATH, "trip", "wattHoursConsumed", trip_A.wattHoursConsumed);
    updateTableValue(SETTINGS_FILEPATH, "trip", "wattHoursRegenerated", trip_A.wattHoursRegenerated);
    updateTableValue(SETTINGS_FILEPATH, "trip_B", "distance", trip_B.distance);
    updateTableValue(SETTINGS_FILEPATH, "trip_B", "wattHoursConsumed", trip_B.wattHoursConsumed);
    updateTableValue(SETTINGS_FILEPATH, "trip_B", "wattHoursRegenerated", trip_B.wattHoursRegenerated);
    updateTableValue(SETTINGS_FILEPATH, "battery", "ampHoursUsed", battery.ampHoursUsed);
    updateTableValue(SETTINGS_FILEPATH, "battery", "ampHoursFullyCharged", battery.ampHoursFullyCharged);
    updateTableValue(SETTINGS_FILEPATH, "battery", "ampHoursFullyChargedWhenNew", battery.ampHoursFullyChargedWhenNew);
    updateTableValue(SETTINGS_FILEPATH, "battery", "ampHoursUsedLifetime", battery.ampHoursUsedLifetime);
    updateTableValue(SETTINGS_FILEPATH, "battery", "wattHoursUsed", battery.wattHoursUsed);
    updateTableValue(SETTINGS_FILEPATH, "battery", "wattHoursFullyDischarged", battery.wattHoursFullyDischarged);
    updateTableValue(SETTINGS_FILEPATH, "settings", "batteryPercentageVoltageBased", settings.batteryPercentageVoltageBased);
    updateTableValue(SETTINGS_FILEPATH, "settings", "regenerativeBraking", settings.regenerativeBraking);
    updateTableValue(SETTINGS_FILEPATH, "settings", "minimizeDrivetrainBacklash", settings.minimizeDrivetrainBacklash);
    updateTableValue(SETTINGS_FILEPATH, "PP", "setProfile", PP.getProfile());

    for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
        std::print("{}\n", PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));

        for (int var = 0; var < PP_VALS::VALS_COUNT; var++) {
            double ret = PP.get(profile, var);

            updateTableValue(SETTINGS_FILEPATH,
                             PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)).c_str(),
                             PP_VALS_TO_STRING.at(static_cast<PP_VALS>(var)).c_str(),
                             ret);
        }

        std::print("\n");
    }

    toSendExtra.append(std::format("{};Settings and variables were saved;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
}

void setMcconfFromCurrentProfile() {
    VESC.data_mcconf.l_current_min_scale = PP.get(PP.getProfile(), PP_VALS::L_CURRENT_MIN_SCALE);
    VESC.data_mcconf.l_current_max_scale = PP.get(PP.getProfile(), PP_VALS::L_CURRENT_MAX_SCALE);
    VESC.data_mcconf.l_min_erpm = PP.get(PP.getProfile(), PP_VALS::L_MIN_ERPM) / 1000.0 * 1.028777545848526;
    VESC.data_mcconf.l_max_erpm = PP.get(PP.getProfile(), PP_VALS::L_MAX_ERPM) / 1000.0 * 1.028777545848526;
    VESC.data_mcconf.l_min_duty = PP.get(PP.getProfile(), PP_VALS::L_MIN_DUTY);
    VESC.data_mcconf.l_max_duty = PP.get(PP.getProfile(), PP_VALS::L_MAX_DUTY);
    VESC.data_mcconf.l_watt_min = PP.get(PP.getProfile(), PP_VALS::L_WATT_MIN);
    VESC.data_mcconf.l_watt_max = PP.get(PP.getProfile(), PP_VALS::L_WATT_MAX);
    VESC.data_mcconf.l_in_current_min = PP.get(PP.getProfile(), PP_VALS::L_IN_CURRENT_MIN);
    VESC.data_mcconf.l_in_current_max = PP.get(PP.getProfile(), PP_VALS::L_IN_CURRENT_MAX);
    VESC.data_mcconf.name = PROFILE_TO_STRING.at(static_cast<PROFILE>(PP.getProfile()));
    VESC.setMcconfTempValues();
}

// ####### Thread Functions #######
// ####### Thread Functions #######
// ####### Thread Functions #######

void uptimeCounterFunction() {
    std::print("[uptimeThread] Started uptime counting\n");
    static std::chrono::duration<double, std::micro> usElapsed;
    while (!done) {
        auto t1 = std::chrono::high_resolution_clock::now();
        uptimeInSeconds += usElapsed.count() / 1000000.0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        usElapsed = std::chrono::high_resolution_clock::now() - t1;
    }
}

enum STATE {
    POWER_OFF_OR_CHARGING = 0,
    THROTTLE = 1,
    BRAKING = 2,
    THROTTLE_TRANSITION_TO_BRAKING = 3,
    BRAKING_TRANSITION_TO_THROTTLE = 4,
};

void throttleFunction() {
    std::printf("[throttleThread] Started thread\n");
    float throttleCurrentToApply = 0.0;
    float brakingCurrentToApply = 0.0;
    static float minCurrent = 5.5;
    static float initialShockTransitionTime = 110.0;
    static float realThrottleTransitionTime = 90.0;
    static float throttleToBrakeTransitionTime = 40.0;
    static float realBrakeTransitionTime = 20.0;
    int state = STATE::POWER_OFF_OR_CHARGING;
    valueTransition.throttleReal.start();

    while (!done) {
        if (fcntl(uartVESC.fd, F_GETFD) != -1) {
            // TODO: settings.regenerativeBraking

            float throttleCurrent = clampValue(
                                            throttleMap.map(throttleLevel),
                                            maxCurrentAtRPM(VESC.data.rpm / motor.magnetPairs)
                                            );

            float brakeCurrent = brakeMap.map(brakeLevel);

            if (!powerOn || battery.charging) {
                state = STATE::POWER_OFF_OR_CHARGING;
            }

            switch (state) {
                case STATE::POWER_OFF_OR_CHARGING:
                    VESC.setCurrent(0.0);

                    if (powerOn && !battery.charging && speed_kmh == 0.0) {
                        state = STATE::THROTTLE;
                    }
                    break;

                case STATE::THROTTLE:
                    if (brakeLevel > 0.0) {
                        state = STATE::THROTTLE_TRANSITION_TO_BRAKING;
                        valueTransition.throttleToBrake.start();
                        break;
                    }

                    if (throttleLevel == 0.0) {
                        if (speed_kmh > 2.0) {
                            throttleCurrentToApply = minCurrent;
                        } else {
                            valueTransition.throttleShockCurrent.start();
                            throttleCurrentToApply = 0.0;
                        }

                        VESC.setCurrent(throttleCurrentToApply);
                        break;
                    }

                    if (throttleLevel > 0.0) {
                        if (valueTransition.throttleShockCurrent.timer.getTime_ms_now() < initialShockTransitionTime) {
                            throttleCurrentToApply = valueTransition.throttleShockCurrent.getValueDifference(0.0, minCurrent, initialShockTransitionTime);
                            valueTransition.throttleReal.start();
                        } else {
                            if (throttleCurrent < minCurrent)
                                throttleCurrentToApply = minCurrent;
                            else {
                                if (valueTransition.throttleReal.timer.getTime_ms_now() < realThrottleTransitionTime) {
                                    throttleCurrentToApply = valueTransition.throttleReal.getValueDifference(minCurrent, throttleCurrent, realThrottleTransitionTime);
                                } else {
                                    throttleCurrentToApply = throttleCurrent;
                                }
                            }
                        }
                    }

                    VESC.setCurrent(throttleCurrentToApply);
                    break;

                case STATE::BRAKING:
                    if (brakeLevel == 0.0 && (throttleLevel != 0.0 || settings.minimizeDrivetrainBacklash)) {
                        state = STATE::BRAKING_TRANSITION_TO_THROTTLE;
                        break;
                    }

                    if (valueTransition.toRealBrake.timer.getTime_ms_now() < realBrakeTransitionTime) {
                        brakingCurrentToApply = valueTransition.toRealBrake.getValueDifference(0.0, brakeCurrent, realBrakeTransitionTime);

                        VESC.setBrakeCurrent(brakingCurrentToApply);
                    } else {
                        VESC.setBrakeCurrent(brakeCurrent);
                    }

                    break;

                case STATE::THROTTLE_TRANSITION_TO_BRAKING:
                    static float _currentToApply;

                    if (valueTransition.throttleToBrake.timer.getTime_ms_now() < throttleToBrakeTransitionTime) {
                        _currentToApply = valueTransition.throttleToBrake.getValueDifference(throttleCurrentToApply, 0.0, throttleToBrakeTransitionTime);

                        VESC.setCurrent(_currentToApply);
                    } else {
                        state = STATE::BRAKING;
                        valueTransition.toRealBrake.start();
                    }

                    break;

                case STATE::BRAKING_TRANSITION_TO_THROTTLE:
                    valueTransition.throttleShockCurrent.start();

                    state = STATE::THROTTLE;
                    break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } // while()
}

void vescValueProcessingFunction() {
    std::printf("[vescValueProcessingThread] Started thread\n");
    while (!done) {
        if (VESC.getVescValues()) {
            static double tachometer_abs_previous;
            static double tachometer_abs_diff;
            static double distanceDiff;

            // if the previous measurement is bigger than the current, that means
            // that the VESC was probably powered off and on, so the stats got reset...
            // So this makes sure that we do not make a tachometer_abs_diff thats suddenly a REALLY
            // large number and therefore screw up our distance measurement
            if (tachometer_abs_previous > VESC.data.tachometerAbs) {
                tachometer_abs_previous = VESC.data.tachometerAbs;
            }

            // prevent the diff to be something extremely big
            if ((VESC.data.tachometerAbs - tachometer_abs_previous) >= 1000) {
                tachometer_abs_previous = VESC.data.tachometerAbs;
            }

            if (tachometer_abs_previous < VESC.data.tachometerAbs) {
                tachometer_abs_diff = VESC.data.tachometerAbs - tachometer_abs_previous;
                tachometer_abs_previous = VESC.data.tachometerAbs;

                distanceDiff = ((tachometer_abs_diff / (double)motor.poles) / (double)wheel.gear_ratio) * (double)wheel.diameter * 3.14159265 / 100000.0; // divide by 100000 for trip distance to be in kilometers

                trip_A.distance += distanceDiff;
                trip_B.distance += distanceDiff;
                odometer.distance += distanceDiff;
                estimatedRange.distance += distanceDiff;
            }

            motor_rpm = (VESC.data.rpm / (float)motor.magnetPairs);
            speed_kmh = (motor_rpm / wheel.gear_ratio) * wheel.diameter * 3.14159265f * 60.0f/*minutes*/ / 100000.0f/*1 km in cm*/;
        }
    }
}

void IPCReadFunction() {
    std::print("[IPCreadThread] Started thread\n");
    std::print("[IPC] Waiting for client to connect\n");
    if (IPC.createClientSocket() != 0) {
        std::print("[IPC] Client failed to connect!\n");
    }

    while(!done) {
        std::string toSend;
        std::string whatWasRead;
        std::string listOfProfiles;
        whatWasRead = IPC.read();

        if (sizeof(whatWasRead.data()) > 1) {
            auto readStringPacket = split(whatWasRead, '\n');

            if (!readStringPacket.empty())
            for (int i = 0; i < (int)readStringPacket.size(); i++) {
                auto packet = split(readStringPacket[i], ';');

                if (!packet.empty()) {
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
                            commAddValue(&toSend, battery.currentForFrontend, 4);
                            commAddValue(&toSend, battery.watts, 1);
                            commAddValue(&toSend, battery.wattHoursUsed, 15);
                            commAddValue(&toSend, battery.wattHoursFullyDischarged, 15);
                            commAddValue(&toSend, battery.ampHoursUsed, 6);
                            commAddValue(&toSend, battery.ampHoursUsedLifetime, 2);
                            commAddValue(&toSend, battery.ampHoursFullyCharged, 2);
                            commAddValue(&toSend, battery.ampHoursFullyChargedWhenNew, 2);
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
                            commAddValue(&toSend, odometer.distance, 7);
                            commAddValue(&toSend, trip_A.distance, 15);
                            commAddValue(&toSend, trip_A.wattHoursUsed, 15);
                            commAddValue(&toSend, trip_A.wattHoursConsumed, 15);
                            commAddValue(&toSend, -(trip_A.wattHoursRegenerated), 15);
                            commAddValue(&toSend, trip_B.distance, 15);
                            commAddValue(&toSend, trip_B.wattHoursUsed, 15);
                            commAddValue(&toSend, trip_B.wattHoursConsumed, 15);
                            commAddValue(&toSend, -(trip_B.wattHoursRegenerated), 15);
                            commAddValue(&toSend, VESC.data.avgMotorCurrent, 1);
                            commAddValue(&toSend, VESC.data.dutyCycleNow * 100.0, 1); // value is now between 0 and 100
                            commAddValue(&toSend, estimatedRange.WhPerKm, 15);
                            commAddValue(&toSend, estimatedRange.distance, 15);
                            commAddValue(&toSend, estimatedRange.range, 15);
                            commAddValue(&toSend, VESC.data.tempMotor, 1);
                            commAddValue(&toSend, VESC.data.tempMosfet, 1);
                            commAddValue(&toSend, uptimeInSeconds, 0);
                            commAddValue(&toSend, whileLoopUsElapsed.count(), 0);
                            commAddValue(&toSend, 1100, 0); // timeCore1
                            commAddValue(&toSend, acceleration, 1);
                            commAddValue(&toSend, powerOn, 0);
                            commAddValue(&toSend, settings.regenerativeBraking, 0);
                            commAddValue(&toSend, PP.getProfile(), 0);
                            commAddValue(&toSend, settings.minimizeDrivetrainBacklash, 0);

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

                        case COMMAND_ID::RESET_TRIP_A:
                            tripReset(&trip_A);
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
                                commAddValue_string(&toSend, VESC.data_mcconf.name);
                                toSend.append("\n");

                                toSend.append(std::format("{};Latest McConf values retrieved!;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                            } else {
                                toSend.append(std::format("{};Latest McConf values did NOT get retrieved!;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                            }
                            break;

                        case COMMAND_ID::SET_POWER_PROFILE_CUSTOM:
                            PP.setProfile(PROFILE::CUSTOM);

                            PP.set(PROFILE::CUSTOM, PP_VALS::L_CURRENT_MIN_SCALE, getValueFromPacket(packet, 1));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_CURRENT_MAX_SCALE, getValueFromPacket(packet, 2));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MIN_ERPM, getValueFromPacket(packet, 3));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MAX_ERPM, getValueFromPacket(packet, 4));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MIN_DUTY, getValueFromPacket(packet, 5));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MAX_DUTY, getValueFromPacket(packet, 6));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_WATT_MIN, getValueFromPacket(packet, 7));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_WATT_MAX, getValueFromPacket(packet, 8));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_IN_CURRENT_MIN, getValueFromPacket(packet, 9));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_IN_CURRENT_MAX, getValueFromPacket(packet, 10));

                            setMcconfFromCurrentProfile();

                            toSend.append(std::format("{};Custom McConf was set;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
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
                            commAddValue(&toSend, analogReadings.analog6, 15);
                            commAddValue(&toSend, analogReadings.analog7, 15);

                            toSend.append("\n");
                            break;

                        case COMMAND_ID::RESET_TRIP_B:
                            tripReset(&trip_B);
                            toSend.append(std::format("{};Trip B was reset;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                            break;

                        case COMMAND_ID::GET_AVAILABLE_POWER_PROFILES:
                            commAddValue(&toSend, COMMAND_ID::GET_AVAILABLE_POWER_PROFILES, 0);

                            listOfProfiles.clear();
                            for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
                                listOfProfiles.append(PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));
                                listOfProfiles += '\0';
                            }

                            toSend.append(listOfProfiles);
                            toSend.append("\n");
                            break;

                        case COMMAND_ID::SET_POWER_PROFILE:
                            PP.setProfile((int)getValueFromPacket(packet, 1));
                            setMcconfFromCurrentProfile();

                            toSend.append(std::format("{};Power profile was set;\n", static_cast<int>(COMMAND_ID::BACKEND_LOG)));
                            break;

                        case COMMAND_ID::SET_MINIMIZE_DRIVETRAIN_BACKLASH:
                            settings.minimizeDrivetrainBacklash = (bool)getValueFromPacket(packet, 1);

                            break;
                    }
                }
            }// !if packet.empty()
        }

        toSend.append(toSendExtra);
        toSendExtra = "";
        IPC.write(toSend.c_str(), toSend.size());
    }
}

// ####### Setup Functions #######
// ####### Setup Functions #######
// ####### Setup Functions #######

void setupTOML() {
    // TODO: if settings.toml doesnt exist, create it
    tbl = toml::parse_file(SETTINGS_FILEPATH);

    // values
    odometer.distance                       = tbl["odometer"]["distance"].value_or<double>(-1);
    trip_A.distance                         = tbl["trip"]["distance"].value_or<double>(-1);
    trip_A.wattHoursConsumed                = tbl["trip"]["wattHoursConsumed"].value_or<double>(-1);
    trip_A.wattHoursRegenerated             = tbl["trip"]["wattHoursRegenerated"].value_or<double>(-1);
    trip_B.distance                         = tbl["trip_B"]["distance"].value_or<double>(-1);
    trip_B.wattHoursConsumed                = tbl["trip_B"]["wattHoursConsumed"].value_or<double>(-1);
    trip_B.wattHoursRegenerated             = tbl["trip_B"]["wattHoursRegenerated"].value_or<double>(-1);
    battery.ampHoursUsed                    = tbl["battery"]["ampHoursUsed"].value_or<double>(-1);
    battery.ampHoursFullyCharged            = tbl["battery"]["ampHoursFullyCharged"].value_or<double>(-1);
    battery.ampHoursFullyChargedWhenNew     = tbl["battery"]["ampHoursFullyChargedWhenNew"].value_or<double>(-1);
    battery.ampHoursUsedLifetime            = tbl["battery"]["ampHoursUsedLifetime"].value_or<double>(-1);
    battery.wattHoursUsed                   = tbl["battery"]["wattHoursUsed"].value_or<double>(-1);
    battery.wattHoursFullyDischarged        = tbl["battery"]["wattHoursFullyDischarged"].value_or<double>(-1);
    settings.batteryPercentageVoltageBased  = tbl["settings"]["batteryPercentageVoltageBased"].value_or(0);
    settings.regenerativeBraking            = tbl["settings"]["regenerativeBraking"].value_or(0);
    settings.minimizeDrivetrainBacklash     = tbl["settings"]["minimizeDrivetrainBacklash"].value_or(0);
    PP.setProfile(tbl["PP"]["setProfile"].value_or(0));

    for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
        std::print("{}\n", PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));

        for (int var = 0; var < PP_VALS::VALS_COUNT; var++) {
            double ret = tbl
                            [PROFILE_TO_STRING.at(static_cast<PROFILE>(profile))]
                            [PP_VALS_TO_STRING.at(static_cast<PP_VALS>(var))]
                            .value_or<double>(-1);

            PP.set(profile, var, ret);

            std::print("i={} -> {} -> {}\n", var, PP_VALS_TO_STRING.at(static_cast<PP_VALS>(var)), PP.get(profile, var));
        }

        std::print("\n");
    }
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
        std::print("[MCP23017] failed to initialize\n");
        done = true;
        return;
    }

    // Setup pins
    pinMode(        pinPowerswitch, INPUT);;
}

void setupADC() {
	if (!mcp3004Setup(MCP3008_BASEPIN, MCP3008_SPICHAN)) {
        std::print("[ADC MCP3004] failed to initialize\n");
        done = true;
        return;
    }
}

void setupADC2() {
    int dev0 = wiringPiI2CSetupInterface("/dev/i2c-0", ADS1115_ADDRESS);
	ads1115Setup_fd(ADS1115_BASEPIN, dev0);
    digitalWrite(ADS1115_BASEPIN, 5); // Diff between ch2 and ch3
    digitalWrite(ADS1115_BASEPIN+1, ADS1115_DR_250);
}

void setupVESC() {
    if (uartVESC.begin(B115200) != 0) {
        std::print("[VESC] failed to initialize UART\n");
        done = true;
        return;
    }
    VESC.setSerialPort(&uartVESC);
}

void setupIPC() {
    if (IPC.createServerSocket(8080) != 0) {
        std::print("[IPC] failed to initialize server socket\n");
        done = true;
        return;
    }
}

int main() {
    if (getuid() != 0) {
        std::printf("Run me as root, please :(\n");
        return -1;
    }
    signal(SIGINT, my_handler);

    VESC.data.maxMotorCurrent = 300.0; // TODO: retrieve it from VESC
    maxBrakingCurrent = 100.0;

    std::vector<Point> throttleCurve = {
        {0, 0},
        {8, 8},
        {15, 13},
        {20, 18},
        {30, 30},
        {40, 60},
        {50, 100},
        {75, 200},
        {87, 250},
        {100, 300}
    };
    throttleMap.setCurve(throttleCurve);

    std::vector<Point> brakeCurve = {
        {0, 0},
        {8, 6},
        {15, 10},
        {20, 15},
        {30, 25},
        {40, 35},
        {50, 50},
        {75, 75},
        {87, 87},
        {100, 100}
    };
    brakeMap.setCurve(brakeCurve);

    movingAverages.potThrottle.smoothingFactor = 0.7;
    movingAverages.brakeThrottle.smoothingFactor = 0.7;
    movingAverages.batteryCurrentForFrontend.smoothingFactor = 0.2;
    movingAverages.brakingCurrent.smoothingFactor = 0.1;

    wheel.rpmPerKmh = (1.0 /*km/h*/ * 1000.0 /*meters*/) / ((3.14 * wheel.diameter) * 60 /*minutes*/);
    motor.rpmPerKmh = wheel.rpmPerKmh * wheel.gear_ratio;

    setupIPC();  // IPC
    setupTOML(); // settings
    setupGPIO(); // RPi GPIO
    setupMCP();  // Pin Expander
    setupADC();  // ADC 8ch 10bit
    setupADC2(); // ADC2 1ch 16bit (15bit)
    setupVESC(); // VESC

    setMcconfFromCurrentProfile();

    std::print("[Main] main loop\n");
    while (!done) {
		auto t1 = std::chrono::high_resolution_clock::now();

        // ############
        // # readings #
        // ############

        // Digital
        powerOn = digitalRead(pinPowerswitch);

        // PWM
        pwmWrite(pinPWM_fan, 256); // 0 - 1023

        // Analog
        analogReadings.analog0 = ((float)analogRead(A0_ADC) / 1023.0) * 3.3;
        analogReadings.analog1 = ((float)analogRead(A1_ADC) / 1023.0) * 3.3;
        analogReadings.analog2 = ((float)analogRead(A2_ADC) / 1023.0) * 3.3;
        analogReadings.analog3 = ((float)analogRead(A3_ADC) / 1023.0) * 3.3;
        analogReadings.analog4 = ((float)analogRead(A4_ADC) / 1023.0) * 3.3;
        analogReadings.analog5 = ((float)analogRead(A5_ADC) / 1023.0) * 3.3;
        analogReadings.analog6 = ((float)analogRead(A6_ADC) / 1023.0) * 3.3;
        analogReadings.analog7 = ((float)analogRead(A7_ADC) / 1023.0) * 3.3;
        double batteryCurrentRaw = -analogRead(ADS1115_BASEPIN+5);

        // ##########################
        // # map all these readings #
        // ##########################
        static double mvPerAmp = 1.345;
        double batteryCurrentMv = (batteryCurrentRaw / 32767.0) * 256.0 /* mV */; // 256mV because the PGA gain is set to 16
        battery.current = batteryCurrentMv / mvPerAmp;
        battery.currentForFrontend = movingAverages.batteryCurrentForFrontend.moveAverage(batteryCurrentMv / mvPerAmp);

        // throttleLevel
        static float throttleMinVoltage = 0.95;
        static float throttleMaxVoltage = 2.5;

        // brakeLevel
        static float brakeMinVoltage = 0.845;
        static float brakeMaxVoltage = 2.48;

        // battery.voltage
        static float batteryVoltageR1 = 220000 + 3500 /* +3500 is the correction value */;
        static float batteryVoltageR2 = 4700+1000+1000+1000;
        static float batteryVoltageVMaxInput = 100;
        static float batteryVoltageVMax = (batteryVoltageVMaxInput * batteryVoltageR2) / (batteryVoltageR1 + batteryVoltageR2);

        throttleLevel       = map_f(movingAverages.potThrottle.moveAverage(analogReadings.analog0), throttleMinVoltage, throttleMaxVoltage, 0.0, 100.0);
        brakeLevel          = map_f(movingAverages.brakeThrottle.moveAverage(analogReadings.analog1), brakeMinVoltage, brakeMaxVoltage, 0.0, 100.0);
        battery.voltage     = map_f_nochecks(analogReadings.analog7, 0.0, batteryVoltageVMax, 0.0, batteryVoltageVMaxInput);

        // Power on/off
        static bool powerOn_tmp = false;
        if (powerOn != powerOn_tmp) {
            powerOn_tmp = powerOn;
            if (powerOn) {
                // run code when turned on
                std::printf("[Main] Power on\n");

            }

            if (!powerOn) {
                // run code when turned off
                std::printf("[Main] Power off\n");

            }
        }

        // calculate other stuff
        battery.watts = battery.voltage * battery.current;

        // BATTERY
        static double _batteryAmpsUsedInElapsedTime,     _batteryWattsUsedUsedInElapsedTime;
        static double _batteryAmpHoursUsedInElapsedTime, _batteryWattHoursUsedUsedInElapsedTime;

        _batteryAmpsUsedInElapsedTime = battery.current / (1000000.0 / whileLoopUsElapsed.count());
        _batteryAmpHoursUsedInElapsedTime = _batteryAmpsUsedInElapsedTime / 3600.0;

        _batteryWattsUsedUsedInElapsedTime = (battery.current * battery.voltage) / (1000000.0 / whileLoopUsElapsed.count());
        _batteryWattHoursUsedUsedInElapsedTime = _batteryWattsUsedUsedInElapsedTime / 3600.0;

        battery.ampHoursUsed += _batteryAmpHoursUsedInElapsedTime;
        if (_batteryAmpHoursUsedInElapsedTime >= 0.0) {
            battery.ampHoursUsedLifetime    += _batteryAmpHoursUsedInElapsedTime;
        }

        battery.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime;

        if (!battery.charging && speed_kmh != 0.0) {
            estimatedRange.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime;

            if (_batteryWattHoursUsedUsedInElapsedTime >= 0.0) {
                trip_A.wattHoursConsumed += _batteryWattHoursUsedUsedInElapsedTime;
                trip_B.wattHoursConsumed += _batteryWattHoursUsedUsedInElapsedTime;
            }

            if (_batteryWattHoursUsedUsedInElapsedTime < 0.0) {
                trip_A.wattHoursRegenerated += _batteryWattHoursUsedUsedInElapsedTime;
                trip_B.wattHoursRegenerated += _batteryWattHoursUsedUsedInElapsedTime;
            }
        }

        trip_A.wattHoursUsed = trip_A.wattHoursConsumed + trip_A.wattHoursRegenerated;
        trip_B.wattHoursUsed = trip_B.wattHoursConsumed + trip_B.wattHoursRegenerated;

        static auto timeAmphoursMinVoltage = std::chrono::high_resolution_clock::now();
        static auto timeWaitBeforeChangingToCharging = std::chrono::high_resolution_clock::now();
        static std::chrono::duration<double, std::milli> timeAmphoursMinVoltageMsElapsed;
        static std::chrono::duration<double, std::milli> timeWaitBeforeChangingToChargingMsElapsed;

        // automatically change the battery.charging status to true if its false & we aint braking & the current is less than 0 (meaning it's charging)
        // we don't have dedicated sensing pin for the charger being connected, yet...
        // after that wait one second, just to make sure it's not a fluke, so it doesnt just randomly put it in charging mode because of a small anomaly...
        if (!battery.charging && brakeLevel == 0.0 && battery.current < 0.0) {
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
            battery.percentage = map_f_nochecks(battery.ampHoursUsed, 0.0, battery.ampHoursFullyCharged, 100.0, 0.0);
        }

        if (!battery.charging && (battery.voltage <= battery.amphours_min_voltage) && (battery.current <= 3.0 && battery.current >= 0.0)) {
            timeAmphoursMinVoltageMsElapsed = std::chrono::high_resolution_clock::now() - timeAmphoursMinVoltage;
            if (timeAmphoursMinVoltageMsElapsed.count() >= 5000) {
                battery.ampHoursFullyCharged_tmp = battery.ampHoursUsed; // save ampHourUsed to ampHourRated_tmp...
                                                                // this temporary value will later get applied to the actual
                                                                // ampHourRated variable when the battery is done charging so
                                                                // the estimated range and other stuff doesn't suddenly get screwed
                battery.wattHoursFullyDischarged_tmp = battery.wattHoursUsed;
            }
        } else {
            timeAmphoursMinVoltage = std::chrono::high_resolution_clock::now();
        }

        if (battery.voltage >= battery.amphours_max_voltage) {
            // TODO: use dedicated current sensing for charging
            if (battery.charging && (battery.current <= 0.0 && battery.current >= -0.5)) {
                battery.ampHoursUsed = 0;
                battery.wattHoursUsed = 0;

                if (battery.ampHoursFullyCharged_tmp != 0.0) {
                    battery.ampHoursFullyCharged = battery.ampHoursFullyCharged_tmp;
                }

                if (battery.wattHoursFullyDischarged_tmp != 0.0) {
                    battery.wattHoursFullyDischarged = battery.wattHoursFullyDischarged_tmp;
                }
            }
        }

        // estimatedRange.range
        estimatedRange.WhPerKm = estimatedRange.wattHoursUsed / estimatedRange.distance;
        double tmp = (battery.wattHoursFullyDischarged - battery.wattHoursUsed) / estimatedRange.WhPerKm;
        tmp != tmp ? estimatedRange.range = 0.0 : estimatedRange.range = tmp;

        static double uptimeInSeconds_tmp = 0;
        if ((uptimeInSeconds - uptimeInSeconds_tmp) >= 1800) { // 30 minutes
            uptimeInSeconds_tmp = uptimeInSeconds;

            saveAll();
        }

        static bool threadsInitialized = false;
        if (!threadsInitialized) {
            threads.uptimeCounter = std::thread(uptimeCounterFunction);
            threads.throttle = std::thread(throttleFunction);
            threads.vescValueProcessing = std::thread(vescValueProcessingFunction);
            threads.IPCRead = std::thread(IPCReadFunction);

            while (!threads.IPCRead.joinable()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            threadsInitialized = true;
        }

        whileLoopUsElapsed = std::chrono::high_resolution_clock::now() - t1;
    }

    threads.uptimeCounter.join();
    threads.throttle.join();
    threads.vescValueProcessing.join();
    threads.IPCRead.join();

    return 0;
}
