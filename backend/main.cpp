// high_resolution_clock::now has an overhead of 0.161812 us

// MCP23017 at 1.2MHz I2C
// switches between HIGH and LOW at 6.85kHz
// digitalRead takes around 88.5us
// digitalWrite takes around 73 us

// #define NOT_RPI

#include <cstdio>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <format>
#include <print>
#include <signal.h>
#include <fcntl.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <mcp23017.h>
#include <mcp3004.h>
#include "toml.hpp"

#include "ads1115.hpp"
#include "myUart.hpp"
#include "VescUart/VescUart.h"
#include "server.hpp"
#include "commonUtils.hpp"
#include "comm.h"
#include "inputOffset.h"
#include "map.hpp"
#include "profiles.hpp"
#include "timer.hpp"
#include "valueTransition.hpp"
#include "rollingRangeEstimation.hpp"
#include "messagingUtils.hpp"
#include "loopRateLimiter.hpp"
#include "rampLimiter.hpp"

#define EBIKE_NAME "EBIKE"
#define EBIKE_VERSION "0.2.2"

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
#define pinChargerConnected  A1_EXP
#define pinPWM_fan      12

ServerSocket IPC;
toml::table table;
VescUart    VESC;
MyUart      uartVESC;
ThrottleMap throttleMap;
ThrottleMap brakeMap;
PowerProfiles PP;
RollingRangeEstimation rollingRangeEstimation;
VescUart::dataPackage VESCData;

std::vector<Point> throttleCurve = {
    {0, 0},
    {8, 8},
    {15, 13},
    {20, 18},
    {30, 30},
    {40, 60},
    {50, 100},
    {75, 180},
    {87, 220},
    {100, 250}
};

std::vector<Point> throttleCurveDualVESC = {
    {0, 0},
    {8, 8},
    {15, 15},
    {20, 25},
    {30, 45},
    {40, 70},
    {50, 110},
    {62, 170},
    {75, 250},
    {87, 350},
    {100, 450}
};

std::vector<Point> brakeCurve = {
    {0, 0},
    {8, 8},
    {15, 13},
    {20, 18},
    {30, 30},
    {40, 45},
    {60, 72},
    {75, 105},
    {87, 150},
    {100, 200}
};

struct {
    MovingAverage batteryCurrentForFrontend;
    MovingAverage batteryVoltageForFrontend;
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

struct {
    LoopRateLimiter threadThrottle;
    LoopRateLimiter threadVescValueProcessing;
} loopRateLimiter;

// TODO: do not hardcode filepaths
// TODO: if the file doesnt exist, create it
const char* SETTINGS_FILEPATH = "/home/snipex/.config/ebike/backend.toml";
std::chrono::duration<double, std::micro> whileLoopUsElapsed;
float acceleration = 0;
double uptimeInSeconds = 0;
float motor_rpm = 0;
float speed_kmh = 0;
float throttleLevel = 0;
float brakeLevel = 0;
bool  powerOn = false;
bool  done = false;
float maxBrakingCurrent = 200.0;
int throttleLoopRate = 100.0;
std::string toSendExtra;
std::string notes;

// forward declaration
void TOMLSave(toml::table &tbl, const char* filepath);

struct Battery {
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
    double voltageForFrontend;

    double ampHoursUsed;
    double ampHoursUsedLifetime;
    double ampHoursFullyCharged;
    double ampHoursFullyCharged_tmp;
    double ampHoursFullyChargedWhenNew;

    double wattHoursUsed;
    double wattHoursFullyDischarged;
    double wattHoursFullyDischarged_tmp;

    double wattHoursRemaining; // calculated at runtime

    bool charging = false;
};
Battery battery;

struct Trip {
    double distance; // in km
    double wattHoursUsed;
    double wattHoursConsumed;
    double wattHoursRegenerated;

    double range; // calculated at runtime
    double WhPerKm; // calculated at runtime
};
Trip trip_A, trip_B;

struct {
    double trip_distance;    // in km
    double distance;         // in km
} odometer;

struct {
    bool batteryPercentageVoltageBased = 0;
    bool automaticRegenerativeBraking = 0;
    bool minimizeDrivetrainBacklash = 0;
    bool enableDualVESC = 0;
    int secondVESCID = 0;
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
    float rpmPerKmh = 0; // calculated at runtime
} motor;

struct {
    float diameter = 63.0;
    float gear_ratio = 8.90625; // (34/10)*(57/16)
    float rpmPerKmh = 0; // calculated at runtime
} wheel;

void estimatedRangeCalculateStats(Trip *trip, double availableWatthours) {
    trip->WhPerKm = trip->wattHoursUsed / trip->distance;
    double tmp = availableWatthours / trip->WhPerKm;
    tmp != tmp ? trip->range = 0.0 : trip->range = tmp;
}

void tripReset(Trip *trip) {
    trip->distance = 0;
    trip->wattHoursConsumed = 0;
    trip->wattHoursRegenerated = 0;
    trip->range = 0;
    trip->WhPerKm = 0;
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

float maxCurrentAtRPM(float rpm, float maxCurrent) {
    float RPM1 = 360.0;
    float RPM2 = 450.0;
    float currentBeforeRPM1 = 150.0;

    if (rpm <= RPM1) {
        return currentBeforeRPM1;
    }

    if (rpm <= RPM2) {
        // Linear interpolation between 120A at 430 ERPM and 190A at 1000 ERPM
        float slope = (maxCurrent - currentBeforeRPM1) / (RPM2 - RPM1);
        return currentBeforeRPM1 + slope * (rpm - RPM1);
    }

    if (rpm > RPM2) {
        return maxCurrent;
    }

    return 0.0;
}

void setMcconfFromCurrentProfile() {
    int profile = PP.getProfile();

    VESC.data_mcconf.l_current_min_scale = PP.get(profile, PP_VALS::L_CURRENT_MIN_SCALE);
    VESC.data_mcconf.l_current_max_scale = PP.get(profile, PP_VALS::L_CURRENT_MAX_SCALE);
    VESC.data_mcconf.l_min_erpm = PP.get(profile, PP_VALS::L_MIN_ERPM) / 1000.0 * 1.234625970641421;
    VESC.data_mcconf.l_max_erpm = PP.get(profile, PP_VALS::L_MAX_ERPM) / 1000.0 * 1.234625970641421;
    VESC.data_mcconf.l_min_duty = PP.get(profile, PP_VALS::L_MIN_DUTY);
    VESC.data_mcconf.l_max_duty = PP.get(profile, PP_VALS::L_MAX_DUTY);
    if (settings.enableDualVESC) {
        VESC.data_mcconf.l_watt_min = PP.get(profile, PP_VALS::L_WATT_MIN) / 2.0;
        VESC.data_mcconf.l_watt_max = PP.get(profile, PP_VALS::L_WATT_MAX) / 2.0;
        VESC.data_mcconf.l_in_current_min = PP.get(profile, PP_VALS::L_IN_CURRENT_MIN) / 2.0;
        VESC.data_mcconf.l_in_current_max = PP.get(profile, PP_VALS::L_IN_CURRENT_MAX) / 2.0;
        VESC.data_mcconf.c_phase_current_max = PP.get(profile, PP_VALS::C_PHASE_CURRENT_MAX) / 2.0;
    } else {
        VESC.data_mcconf.l_watt_min = PP.get(profile, PP_VALS::L_WATT_MIN);
        VESC.data_mcconf.l_watt_max = PP.get(profile, PP_VALS::L_WATT_MAX);
        VESC.data_mcconf.l_in_current_min = PP.get(profile, PP_VALS::L_IN_CURRENT_MIN);
        VESC.data_mcconf.l_in_current_max = PP.get(profile, PP_VALS::L_IN_CURRENT_MAX);
        VESC.data_mcconf.c_phase_current_max = PP.get(profile, PP_VALS::C_PHASE_CURRENT_MAX);
    }
    VESC.data_mcconf.name = PROFILE_TO_STRING.at(static_cast<PROFILE>(profile));

    VESC.setMcconfTempValues();
    if (settings.enableDualVESC)
        VESC.setMcconfTempValues(settings.secondVESCID);
}

// ####### Thread Functions #######
// ####### Thread Functions #######
// ####### Thread Functions #######

void uptimeCounterFunction() {
    std::print("[uptimeThread] Started uptime counting\n");
    auto t1 = std::chrono::high_resolution_clock::now().time_since_epoch();
    while (!done) {
        auto t2 = std::chrono::high_resolution_clock::now().time_since_epoch();
        uptimeInSeconds = std::chrono::duration<double, std::ratio<1>>(t2 - t1).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void VESCApplyThrottle(float current) {
    if (settings.enableDualVESC) {
        VESC.setCurrent(current / 2.0);
        VESC.setCurrent(current / 2.0, settings.secondVESCID);
    } else {
        VESC.setCurrent(current);
    }
}

void VESCApplyBraking(float current) {
    if (settings.enableDualVESC) {
        VESC.setBrakeCurrent(current / 2.0);
        VESC.setBrakeCurrent(current / 2.0, settings.secondVESCID);
    } else {
        VESC.setBrakeCurrent(current);
    }
}

enum STATE {
    POWER_OFF_OR_CHARGING = 0,
    THROTTLE = 1,
    BRAKING = 2,
    THROTTLE_TRANSITION_TO_BRAKING = 3,
    BRAKING_TRANSITION_TO_THROTTLE = 4,
    POWER_OFF_OR_CHARGING_BRAKING = 5,
};

// TODO: move all the throttle functions and maps and other stuff into a centralized MotorController class
void throttleFunction() {
    std::printf("[throttleThread] Started thread\n");
    float throttleCurrentToApply = 0.0;
    float brakingCurrentToApply = 0.0;
    float minCurrent = 8.0;
    float initialShockTransitionTime = 80.0;
    float realThrottleTransitionTime = 60.0;
    float throttleToBrakeTransitionTime = 40.0;
    float realBrakeTransitionTime = 20.0;
    float automaticRegenerativeBrakingCurrent = 20.0; // 20A
    float minSpeedKmh = 2.0; // used for automaticRegenerativeBraking and throttle transitioning
    int state = STATE::POWER_OFF_OR_CHARGING;
    brakeMap.setCurve(brakeCurve);

    valueTransition.throttleReal.start();

    loopRateLimiter.threadThrottle.setRate(throttleLoopRate);
    while (!done) {
        loopRateLimiter.threadThrottle.start();

        if (settings.enableDualVESC) {
            throttleMap.setCurve(throttleCurveDualVESC);
        } else {
            throttleMap.setCurve(throttleCurve);
        }

        static RampLimiter throttleRamp;
        float throttleClampCurrent = settings.enableDualVESC ? VESC.data_mcconf.c_phase_current_max * 2.0 : VESC.data_mcconf.c_phase_current_max;
        float throttleCurrent = throttleRamp.getValue(
                                    clampValue(
                                            throttleMap.map(throttleLevel),
                                            throttleClampCurrent
                                    ),
                                    100,
                                    20,
                                    5
                                );

        static RampLimiter brakeRamp;
        float brakeCurrent = brakeRamp.getValue(
                                clampValue(
                                        brakeMap.map(brakeLevel),
                                        maxBrakingCurrent
                                ),
                                100,
                                40,
                                15
                             );

        if (!powerOn || battery.charging) {
            if (brakeLevel > 0.0) {
                state = STATE::POWER_OFF_OR_CHARGING_BRAKING;
            } else {
                state = STATE::POWER_OFF_OR_CHARGING;
            }
        }

        switch (state) {
            case STATE::POWER_OFF_OR_CHARGING:
                VESCApplyThrottle(0.0);
                if (powerOn && !battery.charging) {
                    state = STATE::THROTTLE;
                }
                break;

            case STATE::THROTTLE:
                if (!settings.minimizeDrivetrainBacklash) {

                    if (settings.automaticRegenerativeBraking && throttleLevel == 0.0 && speed_kmh > minSpeedKmh) {
                        state = STATE::THROTTLE_TRANSITION_TO_BRAKING;
                        valueTransition.throttleToBrake.start();
                        break;
                    }

                    if (brakeLevel == 0.0) {
                        VESCApplyThrottle(throttleCurrent);
                    } else {
                        VESCApplyBraking(brakeCurrent);
                    }
                    break;
                }

                if (brakeLevel > 0.0) {
                    state = STATE::THROTTLE_TRANSITION_TO_BRAKING;
                    valueTransition.throttleToBrake.start();
                    break;
                }

                if (throttleLevel == 0.0) {
                    if (speed_kmh > minSpeedKmh) {
                        throttleCurrentToApply = minCurrent;

                        if (settings.automaticRegenerativeBraking) {
                            state = STATE::THROTTLE_TRANSITION_TO_BRAKING;
                            valueTransition.throttleToBrake.start();
                        }
                    } else {
                        valueTransition.throttleShockCurrent.start();
                        throttleCurrentToApply = 0.0;
                    }

                    VESCApplyThrottle(throttleCurrentToApply);
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

                VESCApplyThrottle(throttleCurrentToApply);
                break;

            case STATE::BRAKING:
                static float _brakeCurrent;
                if (brakeLevel == 0.0 && (throttleLevel != 0.0 || speed_kmh <= minSpeedKmh || (settings.minimizeDrivetrainBacklash && !settings.automaticRegenerativeBraking))) {
                    state = STATE::BRAKING_TRANSITION_TO_THROTTLE;
                    break;
                }

                if (settings.automaticRegenerativeBraking) {
                    _brakeCurrent = automaticRegenerativeBrakingCurrent;

                    if (brakeCurrent > automaticRegenerativeBrakingCurrent) {
                        _brakeCurrent += brakeCurrent - automaticRegenerativeBrakingCurrent;
                    }
                } else {
                    _brakeCurrent = brakeCurrent;
                }

                if (valueTransition.toRealBrake.timer.getTime_ms_now() < realBrakeTransitionTime) {
                    brakingCurrentToApply = valueTransition.toRealBrake.getValueDifference(0.0, _brakeCurrent, realBrakeTransitionTime);

                    VESCApplyBraking(brakingCurrentToApply);
                } else {
                    VESCApplyBraking(_brakeCurrent);
                }

                break;

            case STATE::THROTTLE_TRANSITION_TO_BRAKING:
                static float _currentToApply;

                if (valueTransition.throttleToBrake.timer.getTime_ms_now() < throttleToBrakeTransitionTime) {
                    _currentToApply = valueTransition.throttleToBrake.getValueDifference(throttleCurrentToApply, 0.0, throttleToBrakeTransitionTime);

                    VESCApplyThrottle(_currentToApply);
                } else {
                    state = STATE::BRAKING;
                    valueTransition.toRealBrake.start();
                }

                break;

            case STATE::BRAKING_TRANSITION_TO_THROTTLE:
                valueTransition.throttleShockCurrent.start();

                state = STATE::THROTTLE;
                break;

            case STATE::POWER_OFF_OR_CHARGING_BRAKING:
                VESCApplyBraking(brakeCurrent);
                break;
        }

        loopRateLimiter.threadThrottle.end();
    } // while()
}

void vescValueProcessingFunction() {
    std::printf("[vescValueProcessingThread] Started thread\n");
    Timer timerAcceleration;
    timerAcceleration.start();
    float speed_kmh_previous = 0;
    while (!done) {
        if (VESC.getVescValues()) {
            loopRateLimiter.threadVescValueProcessing.start();
            VescUart::dataPackage tempData = VESC.data;

            if (settings.enableDualVESC && VESC.getVescValues(settings.secondVESCID)) {
                tempData.ampHours += VESC.data.ampHours;
                tempData.ampHoursCharged += VESC.data.ampHoursCharged;
                tempData.avgCurrentDAxis += VESC.data.avgCurrentDAxis;
                tempData.avgCurrentQAxis += VESC.data.avgCurrentQAxis;
                tempData.avgInputCurrent += VESC.data.avgInputCurrent;
                tempData.avgMotorCurrent += VESC.data.avgMotorCurrent;
                tempData.dutyCycleNow += VESC.data.dutyCycleNow;
                tempData.dutyCycleNow = tempData.dutyCycleNow / 2.0;
                tempData.wattHours += VESC.data.wattHours;
                tempData.wattHoursCharged += VESC.data.wattHoursCharged;
                if (tempData.tempMotor < VESC.data.tempMotor) {
                    tempData.tempMotor = VESC.data.tempMosfet;
                }
                if (tempData.tempMosfet < VESC.data.tempMosfet) {
                    tempData.tempMosfet = VESC.data.tempMosfet;
                }
            }
            VESCData = tempData;

            static double tachometer_abs_previous;
            static double tachometer_abs_diff;
            static double distanceDiff;

            // if the previous measurement is bigger than the current, that means
            // that the VESC was probably powered off and on, so the stats got reset...
            // So this makes sure that we do not make a tachometer_abs_diff thats suddenly a REALLY
            // large number and therefore screw up our distance measurement
            if (tachometer_abs_previous > VESCData.tachometerAbs) {
                tachometer_abs_previous = VESCData.tachometerAbs;
            }

            // prevent the diff to be something extremely big
            if ((VESCData.tachometerAbs - tachometer_abs_previous) >= 1000) {
                tachometer_abs_previous = VESCData.tachometerAbs;
            }

            if (tachometer_abs_previous < VESCData.tachometerAbs) {
                tachometer_abs_diff = VESCData.tachometerAbs - tachometer_abs_previous;
                tachometer_abs_previous = VESCData.tachometerAbs;

                distanceDiff = ((tachometer_abs_diff / (double)motor.poles) / (double)wheel.gear_ratio) * (double)wheel.diameter * 3.14159265 / 100000.0; // divide by 100000 for trip distance to be in kilometers

                trip_A.distance += distanceDiff;
                trip_B.distance += distanceDiff;
                odometer.distance += distanceDiff;
                rollingRangeEstimation.addDeltaDistance(distanceDiff);

            }

            motor_rpm = (VESCData.rpm / (float)motor.magnetPairs);
            speed_kmh = (motor_rpm / wheel.gear_ratio) * wheel.diameter * 3.14159265f * 60.0f/*minutes*/ / 100000.0f/*1 km in cm*/;

            double timeNow = timerAcceleration.getTime_ms_now();
            if (timeNow >= 400.0 /*ms*/) {
                timerAcceleration.start();

                acceleration = (speed_kmh - speed_kmh_previous) * (1000.0 / timeNow);
                speed_kmh_previous = speed_kmh;
            }

            loopRateLimiter.threadVescValueProcessing.end();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void IPCReadFunction() {
    std::print("[IPCreadThread] Started thread\n");

    while(!done) {
        std::string toSend;
        std::string whatWasRead;
        std::string listOfProfiles;
        whatWasRead = IPC.read();

        if (sizeof(whatWasRead.data()) > 1) {
            auto readStringPacket = msg::split(whatWasRead, msg::messageEnd);

        if (!readStringPacket.empty())
            for (int i = 0; i < (int)readStringPacket.size(); i++) {
                auto packet = msg::split(readStringPacket[i], ";");

                int index = 1;

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
                            msg::start(toSend, COMMAND_ID::GET_BATTERY);
                            msg::addValue(toSend, battery.voltageForFrontend, 2);
                            msg::addValue(toSend, battery.currentForFrontend, 4);
                            msg::addValue(toSend, battery.watts, 1);
                            msg::addValue(toSend, battery.wattHoursUsed, 15);
                            msg::addValue(toSend, battery.wattHoursFullyDischarged, 15);
                            msg::addValue(toSend, battery.ampHoursUsed, 6);
                            msg::addValue(toSend, battery.ampHoursUsedLifetime, 2);
                            msg::addValue(toSend, battery.ampHoursFullyCharged, 2);
                            msg::addValue(toSend, battery.ampHoursFullyChargedWhenNew, 2);
                            msg::addValue(toSend, battery.percentage, 1);
                            msg::addValue(toSend, battery.voltage_min, 1);
                            msg::addValue(toSend, battery.voltage_max, 1);
                            msg::addValue(toSend, battery.voltage_nominal, 1);
                            msg::addValue(toSend, battery.amphours_min_voltage, 1);
                            msg::addValue(toSend, battery.amphours_max_voltage, 1);
                            msg::addValue(toSend, battery.charging, 0);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::ARE_YOU_ALIVE:
                            msg::start(toSend, COMMAND_ID::ARE_YOU_ALIVE);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::GET_STATS:
                            msg::start(toSend, COMMAND_ID::GET_STATS);
                            msg::addValue(toSend, speed_kmh, 1);
                            msg::addValue(toSend, motor_rpm, 0);
                            msg::addValue(toSend, motor.rpmPerKmh, 7);
                            msg::addValue(toSend, motor.magnetPairs, 0);
                            msg::addValue(toSend, odometer.distance, 7);
                            msg::addValue(toSend, trip_A.distance, 15);
                            msg::addValue(toSend, trip_A.wattHoursUsed, 15);
                            msg::addValue(toSend, trip_A.wattHoursConsumed, 15);
                            msg::addValue(toSend, -(trip_A.wattHoursRegenerated), 15);
                            msg::addValue(toSend, trip_A.range, 15);
                            msg::addValue(toSend, trip_B.distance, 15);
                            msg::addValue(toSend, trip_B.wattHoursUsed, 15);
                            msg::addValue(toSend, trip_B.wattHoursConsumed, 15);
                            msg::addValue(toSend, -(trip_B.wattHoursRegenerated), 15);
                            msg::addValue(toSend, trip_B.range, 15);
                            msg::addValue(toSend, VESCData.avgMotorCurrent, 1);
                            msg::addValue(toSend, VESCData.avgCurrentDAxis, 1);
                            msg::addValue(toSend, VESCData.avgCurrentQAxis, 1);
                            msg::addValue(toSend, VESCData.dutyCycleNow * 100.0, 1); // value is now between 0 and 100
                            msg::addValue(toSend, VESCData.tempMotor, 1);
                            msg::addValue(toSend, VESCData.tempMosfet, 1);
                            msg::addValue(toSend, uptimeInSeconds, 0);
                            msg::addValue(toSend, whileLoopUsElapsed.count() / 1000.0, 1);
                            msg::addValue(toSend, 1000.0 / loopRateLimiter.threadThrottle.getLoopRate(), 1);
                            msg::addValue(toSend, 1000.0 / loopRateLimiter.threadVescValueProcessing.getLoopRate(), 1);
                            msg::addValue(toSend, acceleration, 1);
                            msg::addValue(toSend, powerOn, 0);
                            msg::addValue(toSend, settings.automaticRegenerativeBraking, 0);
                            msg::addValue(toSend, PP.getProfile(), 0);
                            msg::addValue(toSend, settings.minimizeDrivetrainBacklash, 0);
                            msg::addValue(toSend, rollingRangeEstimation.getRange(), 15);
                            msg::addValue(toSend, rollingRangeEstimation.getWhPerKm(), 15);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::SET_ODOMETER:
                            odometer.distance = msg::getValueFromSplit_double(packet, index);

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Odometer was set to: {} km", odometer.distance);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::SAVE_PREFERENCES:
                            TOMLSave(table, SETTINGS_FILEPATH);

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Preferences were manually saved");
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::RESET_TRIP_A:
                            tripReset(&trip_A);

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Trip was reset");
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::GET_FW:
                            msg::start(toSend, COMMAND_ID::GET_FW);
                            msg::addString(toSend, EBIKE_NAME);
                            msg::addString(toSend, EBIKE_VERSION);
                            msg::addString(toSend, "{} {}", __DATE__, __TIME__);
                            msg::end(toSend);
                            break;

                        // case COMMAND_ID::PING:
                        //     display.ping();
                        //     break;

                        // case COMMAND_ID::TOGGLE_FRONT_LIGHT:

                        //     break;

                        case COMMAND_ID::SET_AMPHOURS_USED_LIFETIME:
                            battery.ampHoursUsedLifetime = msg::getValueFromSplit_double(packet, index);
                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Amphours used (Lifetime) was set to: {} Ah", battery.ampHoursUsedLifetime);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::GET_VESC_MCCONF: {
                            VescUart::mcconf_t _mcconf;
                            int VESC1Success = false;
                            int VESC2Success = false;

                            if (VESC.getMcconfTempValues()) {
                                _mcconf = VESC.data_mcconf;

                                VESC1Success = true;
                            }

                            if (settings.enableDualVESC) {
                                if (VESC.getMcconfTempValues(settings.secondVESCID)) {
                                    _mcconf.l_current_min_scale = (_mcconf.l_current_min_scale + VESC.data_mcconf.l_current_min_scale) / 2.0;
                                    _mcconf.l_current_max_scale = (_mcconf.l_current_max_scale + VESC.data_mcconf.l_current_max_scale) / 2.0;
                                    _mcconf.l_min_erpm = (_mcconf.l_min_erpm + VESC.data_mcconf.l_min_erpm) / 2.0;
                                    _mcconf.l_max_erpm = (_mcconf.l_max_erpm + VESC.data_mcconf.l_max_erpm) / 2.0;
                                    _mcconf.l_min_duty = (_mcconf.l_min_duty + VESC.data_mcconf.l_min_duty) / 2.0;
                                    _mcconf.l_max_duty = (_mcconf.l_max_duty + VESC.data_mcconf.l_max_duty) / 2.0;
                                    _mcconf.l_watt_min += VESC.data_mcconf.l_watt_min;
                                    _mcconf.l_watt_max += VESC.data_mcconf.l_watt_max;
                                    _mcconf.l_in_current_min += VESC.data_mcconf.l_in_current_min;
                                    _mcconf.l_in_current_max += VESC.data_mcconf.l_in_current_max;
                                    //
                                    _mcconf.c_phase_current_max += VESC.data_mcconf.c_phase_current_max;

                                    VESC2Success = true;
                                }
                            }

                            if ((VESC1Success && !settings.enableDualVESC)|| (VESC1Success && VESC2Success && settings.enableDualVESC)) {
                                msg::start(toSend, COMMAND_ID::GET_VESC_MCCONF);
                                msg::addValue(toSend, _mcconf.l_current_min_scale, 7);
                                msg::addValue(toSend, _mcconf.l_current_max_scale, 7);
                                msg::addValue(toSend, _mcconf.l_min_erpm, 7);
                                msg::addValue(toSend, _mcconf.l_max_erpm, 7);
                                msg::addValue(toSend, _mcconf.l_min_duty, 7);
                                msg::addValue(toSend, _mcconf.l_max_duty, 7);
                                msg::addValue(toSend, _mcconf.l_watt_min, 7);
                                msg::addValue(toSend, _mcconf.l_watt_max, 7);
                                msg::addValue(toSend, _mcconf.l_in_current_min, 7);
                                msg::addValue(toSend, _mcconf.l_in_current_max, 7);
                                msg::addString(toSend, "{}", _mcconf.name);
                                msg::addValue(toSend, _mcconf.c_phase_current_max, 7);
                                msg::end(toSend);

                                msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                                msg::addString(toSend, "Latest McConf values retrieved");
                                msg::end(toSend);
                            } else {
                                msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                                msg::addString(toSend, "Latest McConf values did NOT get retrieved!");
                                msg::end(toSend);
                            }
                            break;
                        }
                        case COMMAND_ID::SET_POWER_PROFILE_CUSTOM:
                            PP.setProfile(PROFILE::CUSTOM);

                            PP.set(PROFILE::CUSTOM, PP_VALS::L_CURRENT_MIN_SCALE, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_CURRENT_MAX_SCALE, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MIN_ERPM, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MAX_ERPM, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MIN_DUTY, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_MAX_DUTY, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_WATT_MIN, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_WATT_MAX, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_IN_CURRENT_MIN, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::L_IN_CURRENT_MAX, msg::getValueFromSplit(packet, index));
                            PP.set(PROFILE::CUSTOM, PP_VALS::C_PHASE_CURRENT_MAX, msg::getValueFromSplit(packet, index));

                            setMcconfFromCurrentProfile();

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Custom McConf was set!");
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::SET_AMPHOURS_CHARGED:
                            {
                                float newValue = msg::getValueFromSplit(packet, index);

                                battery.ampHoursFullyCharged = newValue;
                                battery.ampHoursFullyCharged_tmp = newValue;

                                msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                                msg::addString(toSend, "Amphours charged was set to: {} Ah", newValue);
                                msg::end(toSend);
                            }
                            break;

                        case COMMAND_ID::TOGGLE_CHARGING_STATE:
                            battery.charging = !battery.charging;

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Charging state was set to: {}", battery.charging);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::SET_AUTOMATIC_REGEN_BRAKING:
                            settings.automaticRegenerativeBraking = (bool)msg::getValueFromSplit(packet, index);

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Regenerative braking state was set to: {}", settings.automaticRegenerativeBraking);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::GET_ANALOG_READINGS:
                            msg::start(toSend, COMMAND_ID::GET_ANALOG_READINGS);
                            msg::addValue(toSend, analogReadings.analog0, 15);
                            msg::addValue(toSend, analogReadings.analog1, 15);
                            msg::addValue(toSend, analogReadings.analog2, 15);
                            msg::addValue(toSend, analogReadings.analog3, 15);
                            msg::addValue(toSend, analogReadings.analog4, 15);
                            msg::addValue(toSend, analogReadings.analog5, 15);
                            msg::addValue(toSend, analogReadings.analog6, 15);
                            msg::addValue(toSend, analogReadings.analog7, 15);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::RESET_TRIP_B:
                            tripReset(&trip_B);

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Trip B was reset");
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::GET_AVAILABLE_POWER_PROFILES:
                            msg::start(toSend, COMMAND_ID::GET_AVAILABLE_POWER_PROFILES);

                            listOfProfiles.clear();
                            for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
                                listOfProfiles.append(PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));
                                listOfProfiles += '\0';
                            }

                            msg::addString(toSend, "{}", listOfProfiles);
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::SET_POWER_PROFILE:
                            PP.setProfile((int)msg::getValueFromSplit(packet, index));
                            setMcconfFromCurrentProfile();

                            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
                            msg::addString(toSend, "Power profile was set");
                            msg::end(toSend);
                            break;

                        case COMMAND_ID::SET_MINIMIZE_DRIVETRAIN_BACKLASH:
                            settings.minimizeDrivetrainBacklash = (bool)msg::getValueFromSplit(packet, index);

                            break;

                        case COMMAND_ID::GET_NOTES:
                            msg::start(toSend, COMMAND_ID::GET_NOTES);
                            msg::addString(toSend, "{}", notes);
                            msg::end(toSend);

                            break;

                        case COMMAND_ID::SET_NOTES:
                            notes = msg::getValueFromSplit_string(packet, index);

                            break;
                    }
                }
            }// !if packet.empty()
        }

        msg::mtx.lock();
            toSend.append(toSendExtra);
            toSendExtra = "";
        msg::mtx.unlock();
        IPC.write(toSend.c_str(), toSend.size());
    }
}

// ####### Setup Functions #######
// ####### Setup Functions #######
// ####### Setup Functions #######

void setupTOML(toml::table &tbl, const char* filepath) {
    tbl = toml::parse_file(filepath);

    // values
    odometer.distance                       = tbl["odometer"]["distance"].value_or<double>(0);
    trip_A.distance                         = tbl["trip"]["distance"].value_or<double>(0);
    trip_A.wattHoursConsumed                = tbl["trip"]["wattHoursConsumed"].value_or<double>(0);
    trip_A.wattHoursRegenerated             = tbl["trip"]["wattHoursRegenerated"].value_or<double>(0);
    trip_B.distance                         = tbl["trip_B"]["distance"].value_or<double>(0);
    trip_B.wattHoursConsumed                = tbl["trip_B"]["wattHoursConsumed"].value_or<double>(0);
    trip_B.wattHoursRegenerated             = tbl["trip_B"]["wattHoursRegenerated"].value_or<double>(0);
    battery.ampHoursUsed                    = tbl["battery"]["ampHoursUsed"].value_or<double>(0);
    battery.ampHoursFullyCharged            = tbl["battery"]["ampHoursFullyCharged"].value_or<double>(0);
    battery.ampHoursFullyChargedWhenNew     = tbl["battery"]["ampHoursFullyChargedWhenNew"].value_or<double>(0);
    battery.ampHoursUsedLifetime            = tbl["battery"]["ampHoursUsedLifetime"].value_or<double>(0);
    battery.wattHoursUsed                   = tbl["battery"]["wattHoursUsed"].value_or<double>(0);
    battery.wattHoursFullyDischarged        = tbl["battery"]["wattHoursFullyDischarged"].value_or<double>(0);
    settings.batteryPercentageVoltageBased  = tbl["settings"]["batteryPercentageVoltageBased"].value_or(0);
    settings.automaticRegenerativeBraking   = tbl["settings"]["automaticRegenerativeBraking"].value_or(0);
    settings.minimizeDrivetrainBacklash     = tbl["settings"]["minimizeDrivetrainBacklash"].value_or(0);
    settings.enableDualVESC                 = tbl["settings"]["enableDualVESC"].value_or(0);
    settings.secondVESCID                 = tbl["settings"]["secondVESCID"].value_or(0);
    PP.setProfile(tbl["PP"]["setProfile"].value_or(0));

    notes                                   = tbl["notes"]["note1"].value_or("");

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

void TOMLSave(toml::table &tbl, const char* filepath) {
    updateTableValue(tbl, "odometer", "distance", odometer.distance);
    updateTableValue(tbl, "trip", "distance", trip_A.distance);
    updateTableValue(tbl, "trip", "wattHoursConsumed", trip_A.wattHoursConsumed);
    updateTableValue(tbl, "trip", "wattHoursRegenerated", trip_A.wattHoursRegenerated);
    updateTableValue(tbl, "trip_B", "distance", trip_B.distance);
    updateTableValue(tbl, "trip_B", "wattHoursConsumed", trip_B.wattHoursConsumed);
    updateTableValue(tbl, "trip_B", "wattHoursRegenerated", trip_B.wattHoursRegenerated);
    updateTableValue(tbl, "battery", "ampHoursUsed", battery.ampHoursUsed);
    updateTableValue(tbl, "battery", "ampHoursFullyCharged", battery.ampHoursFullyCharged);
    updateTableValue(tbl, "battery", "ampHoursFullyChargedWhenNew", battery.ampHoursFullyChargedWhenNew);
    updateTableValue(tbl, "battery", "ampHoursUsedLifetime", battery.ampHoursUsedLifetime);
    updateTableValue(tbl, "battery", "wattHoursUsed", battery.wattHoursUsed);
    updateTableValue(tbl, "battery", "wattHoursFullyDischarged", battery.wattHoursFullyDischarged);
    updateTableValue(tbl, "settings", "batteryPercentageVoltageBased", settings.batteryPercentageVoltageBased);
    updateTableValue(tbl, "settings", "automaticRegenerativeBraking", settings.automaticRegenerativeBraking);
    updateTableValue(tbl, "settings", "minimizeDrivetrainBacklash", settings.minimizeDrivetrainBacklash);
    updateTableValue(tbl, "settings", "enableDualVESC", settings.enableDualVESC);
    updateTableValue(tbl, "settings", "secondVESCID", settings.secondVESCID);
    updateTableValue(tbl, "PP", "setProfile", PP.getProfile());

    updateTableValue(tbl, "notes", "note1", notes);

    for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
        std::print("{}\n", PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));

        for (int var = 0; var < PP_VALS::VALS_COUNT; var++) {
            double ret = PP.get(profile, var);

            updateTableValue(tbl,
                             PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)).c_str(),
                             PP_VALS_TO_STRING.at(static_cast<PP_VALS>(var)).c_str(),
                             ret);
        }

        std::print("\n");
    }

    saveTableToFile(tbl, filepath);

    msg::start(toSendExtra, COMMAND_ID::BACKEND_LOG);
    msg::addString(toSendExtra, "Settings and variables were saved");
    msg::end(toSendExtra);
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
    pinMode(     pinPowerswitch, INPUT);
    pinMode(pinChargerConnected, INPUT);
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
    digitalWrite(ADS1115_BASEPIN+1, ADS1115_DR_128);
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

    movingAverages.batteryCurrentForFrontend.smoothingFactor = 0.35;
    movingAverages.batteryVoltageForFrontend.smoothingFactor = 0.35;

    wheel.rpmPerKmh = (1.0 /*km/h*/ * 1000.0 /*meters*/) / ((3.14 * wheel.diameter) * 60 /*minutes*/) * 100.0 /*?*/;
    motor.rpmPerKmh = wheel.rpmPerKmh * wheel.gear_ratio;

    setupIPC();  // IPC
    setupTOML(table, SETTINGS_FILEPATH); // settings
    #ifndef NOT_RPI
    setupGPIO(); // RPi GPIO
    setupMCP();  // Pin Expander
    setupADC();  // ADC 8ch 10bit
    setupADC2(); // ADC2 1ch 16bit (15bit)
    setupVESC(); // VESC
    #endif

    setMcconfFromCurrentProfile();

    std::print("[Main] main loop\n");
    while (!done) {
		auto t1 = std::chrono::high_resolution_clock::now();

        // ############
        // # readings #
        // ############

        double batteryCurrentRaw = 0.0;
        #ifndef NOT_RPI
        // Digital
        powerOn = digitalRead(pinPowerswitch);
        // battery.charging = digitalRead(pinChargerConnected);

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
        batteryCurrentRaw = -analogRead(ADS1115_BASEPIN+5);
        #endif

        #ifdef NOT_RPI
            if (speed_kmh >= 49) {
                speed_kmh = 0;
            } else if (speed_kmh < 49) {
                speed_kmh++;
            }
            powerOn = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        #endif

        // ##########################
        // # map all these readings #
        // ##########################
        static double mvPerAmp = 1.345;
        double batteryCurrentMv = (batteryCurrentRaw / 32767.0) * 256.0 /* mV */; // 256mV because the PGA gain is set to 16
        battery.current = batteryCurrentMv / mvPerAmp;
        battery.currentForFrontend = movingAverages.batteryCurrentForFrontend.moveAverage(battery.current);

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

        throttleLevel       = map_f(analogReadings.analog0, throttleMinVoltage, throttleMaxVoltage, 0.0, 100.0);
        brakeLevel          = map_f(analogReadings.analog1, brakeMinVoltage, brakeMaxVoltage, 0.0, 100.0);
        battery.voltage     = map_f_nochecks(analogReadings.analog7, 0.0, batteryVoltageVMax, 0.0, batteryVoltageVMaxInput);

        battery.voltageForFrontend = movingAverages.batteryVoltageForFrontend.moveAverage(battery.voltage);

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
        battery.wattHoursRemaining = battery.wattHoursFullyDischarged - battery.wattHoursUsed;
        rollingRangeEstimation.loop(battery.wattHoursRemaining);

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
            rollingRangeEstimation.addDeltaWhUsed(_batteryWattHoursUsedUsedInElapsedTime);

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
        static std::chrono::duration<double, std::milli> timeAmphoursMinVoltageMsElapsed;

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

        estimatedRangeCalculateStats(&trip_A, battery.wattHoursRemaining);
        estimatedRangeCalculateStats(&trip_B, battery.wattHoursRemaining);

        static double uptimeInSeconds_tmp = 0;
        if ((uptimeInSeconds - uptimeInSeconds_tmp) >= 1800) { // 30 minutes
            uptimeInSeconds_tmp = uptimeInSeconds;

            TOMLSave(table, SETTINGS_FILEPATH);
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
