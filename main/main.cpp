#include <stdio.h>
#include <iostream>
#include <print>
#include <sstream>

// Arduino Libraries
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "HardwareSerial.h"
#include "Preferences.h"
#include <Adafruit_ADS1X15.h>
#include <VescUart.h>
#include <Adafruit_SH110X.h>
#include "Adafruit_GFX.h"

#include "driver/adc.h"
#include "driver/ledc.h"

#include "inputOffset.h"
#include "fastGPIO.h"
#include "ebike-utils.h"
#include "timer_u32.h"
#include "MiniPID.h"
#include "map.cpp"

#define COLOR_WHITE SH110X_WHITE
#define COLOR_BLACK SH110X_BLACK

#define EBIKE_NAME "EBIKE"
#define EBIKE_VERSION "0.0.0"

class Display {
private:
    uint32_t timeStart;
    uint32_t timeEnd;

public:
    void ping() {
        timeStart = timer_u32();
    }

    bool isExternalDisplayConnected() {
        timeEnd = timer_u32();
        if (timer_delta_ms(timeEnd - timeStart) > 1000.0) {
            return false;
        } else {
            return true;
        }
    }
};

enum COMMAND_ID {
    GET_BATTERY = 0,
    ARE_YOU_ALIVE = 1,
    GET_STATS = 2,
    RESET_ESTIMATED_RANGE = 3,
    RESET_TRIP = 4,
    READY_FOR_MESSAGE = 5,
    SET_ODOMETER = 6,
    SAVE_PREFERENCES = 7,
    READY_TO_WRITE = 8,
    GET_FW = 9,
    PING = 10,
    TOGGLE_FRONT_LIGHT = 11,
    ESP32_SERIAL_LENGTH = 12,
    SET_AMPHOURS_USED_LIFETIME = 13,
    GET_VESC_MCCONF = 14,
    SET_VESC_MCCONF = 15,
    SET_AMPHOURS_CHARGED = 16,
    ESP32_LOG = 17,
    TOGGLE_CHARGING_STATE = 18,
    TOGGLE_REGEN_BRAKING = 19,
    ESP32_RESTART = 20
};

struct {
    bool automaticGearChanging = 1;
    bool drawDebug = 0;
    bool drawUptime = 1;
    bool printCoreExecutionTime = 0;
    bool disableOptimizedDrawing = 0;
    bool batteryPercentageVoltageBased = 0;
    bool regenerativeBraking = 0;
} settings;

struct {
    float diameter;
    float gear_ratio;
} wheel;

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
    int poles;
    int magnetPairs;
} motor;

bool firsttime_draw = 1;

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

struct {
    int level;
    int maxCurrent;
} gear1, gear2, gear3, gearCurrent;

struct {
    float range;
    float wattHoursUsedOnStart;
    float distance;
    float wattHoursUsed;
    float WhPerKm;
} estimatedRange;

float  _batteryVoltage;

float _batteryWattHoursUsedUsedInElapsedTime;

float  _batteryCurrent;
float _batteryCurrentUsedInElapsedTime;

float batteryampHoursUsed_tmp;

float  _potThrottleVoltage;
float potThrottleVoltage;
float PotThrottleAmpsRequested;

float wh_over_km = 0; // immediate
float wh_over_km_average = 0; // over time
float speed_kmh = 0;
float speed_kmh_previous = 0;
float acceleration = 0;
float motor_rpm = 0;

bool BRAKING = 0;

std::string toSendExtra;

uint32_t timeStartCore1 = 0, timeCore1 = 0;    
uint32_t timeStartCore0 = 0, timeCore0 = 0;    
uint32_t timeStartDisplay = 0; //timeDisplay = 0;
uint32_t timeStartExecEverySecondCore0 = 0, timeExecEverySecondCore0 = 0;
uint32_t timeExecEverySecondCore1 = 0;
uint32_t timeExecEvery100millisecondsCore1 = 0;
uint32_t timeRotorSleep = 0;
uint32_t timeAmphoursMinVoltage = 0;
uint32_t timeWaitBeforeChangingToCharging = 0;
unsigned long timeSavePreferencesStart = millis();

// uptime
float totalSecondsSinceBoot;
int clockSecondsSinceBoot;
int clockMinutesSinceBoot;
int clockHoursSinceBoot;
int clockDaysSinceBoot;

uint64_t totalSecondsSinceBoot_uint64;

// clock
int clockYear = 2025;
int clockMonth = 4;
int clockDay = 24;
int clockHours = 15;
int clockMinutes = 10;
float clockSeconds = 15;
const int daysInJanuary = 31;
const int daysInFebruary = 28;
const int daysInMarch = 31;
const int daysInApril = 30;
const int daysInMay = 31;
const int daysInJune = 30;
const int daysInJuly = 31;
const int daysInAugust = 31;
const int daysInSeptember = 30;
const int daysInOctober = 31;
const int daysInNovember = 30;
const int daysInDecember = 31;

float PotThrottleAdjustment;
float PotThrottleLevelReal;

char text[128];

std::string readString;

InputOffset BatVoltageCorrection;
InputOffset PotThrottleCorrection;

MovingAverage BatVoltageMovingAverage;
MovingAverage PotThrottleMovingAverage;
MovingAverage BatWattMovingAverage;
MovingAverage WhOverKmMovingAverage;
MovingAverage batteryAuxWattsMovingAverage;
MovingAverage batteryAuxMovingAverage;
MovingAverage batteryWattsMovingAverage;
MovingAverage speed_kmhMovingAverage;
MovingAverage wattageMoreSmooth_MovingAverage;

struct {
    MovingAverage brakingCurrent;
} movingAverages;

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3c
Adafruit_SH1106G tft = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

ThrottleMap         throttle;
Display             display;
Preferences         preferences;
Adafruit_ADS1115    dedicatedADC;
VescUart            VESC;

class PreferencesActions {
public:
    void saveOdometer() {
        preferences.putFloat("odometer", (float)odometer.distance);
    }

    void saveBattery() {
        preferences.putFloat("batAhUsed", battery.ampHoursUsed);
        preferences.putFloat("batAhFulDis", battery.ampHoursRated);
        preferences.putFloat("batAhUsedLife", battery.ampHoursUsedLifetime);
    }

    void saveOther() {
        preferences.putBool("regenBrake", settings.regenerativeBraking);
    }

    void saveAll() {
        saveOdometer();
        saveBattery();
        saveOther();
    }
};
PreferencesActions preferenceActions;

double kP = 1, kI = 0.02, kD = 0.5; //kP = 0.1, 0.3 is unstable
MiniPID powerLimiterPID(kP, kI, kD);

#define pinRotor          25                // D25
#define pinBatteryVoltage ADC1_CHANNEL_0    // D36
#define pinPotThrottle    ADC1_CHANNEL_3    // D39 // VN
#define pinButton1  -1
#define pinButton2  -1
#define pinButton3  -1
#define pinButton4  -1
#define pinAdcRdyAlert 33 // D33
#define pinPowerSwitch 27 // D27
#define pinBrake       13 // D13

#define ADC_ATTEN      ADC_ATTEN_DB_12  // Allows reading up to 3.3V
#define ADC_WIDTH      ADC_WIDTH_BIT_12 // 12-bit resolution

// Enabling C++ compile
extern "C" { void app_main(); }

std::vector<std::string> split(const std::string& input, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);
    }

    return result;
}

void commAddValue(std::string* string, double value, int precision) {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;

    string->append(out.str());
    string->append(";");
}

void commAddValue_uint64(std::string* string, uint64_t value, int precision) {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;

    string->append(out.str());
    string->append(";");
}

float getValueFromPacket(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        return std::stof(token[index]);
    }

    std::println("Index out of bounds");
    return -1;
}

uint64_t getValueFromPacket_uint64(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        std::stringstream stream(token[index]);
        uint64_t result;
        stream >> result;
        return result;
    }

    std::println("Index out of bounds");
    return -1;
}

std::string getValueFromPacket_string(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        return token[index];
    }

    std::println("Index out of bounds");
    return "-1";
}

int setDuty_previousDuty;
void setDuty(ledc_channel_t channel, int duty) {
    if (setDuty_previousDuty != duty) { // set the new dutyCycle only if the value is different from the previous one
        setDuty_previousDuty = duty;
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
    }
}

void estimatedRangeReset() {
    estimatedRange.wattHoursUsedOnStart  = battery.wattHoursUsed;
    estimatedRange.distance              = 0.0;
    estimatedRange.wattHoursUsed         = 0.0;
}

void tripReset() {
    trip.distance = 0;
    trip.wattHoursUsed = 0;
}

float maxCurrentAtERPM(int erpm) {
    // 288 ERPM per km/h
    // 48 RPM per km/h
    float ERPM_current1 = 1500.0f; // ((1500/6)/5.7) * 63 * 3.14 * 60 / 100000 = 5.21 km/h
    float ERPM_current2 = 1700.0f; // ((1700/6)/5.7) * 63 * 3.14 * 60 / 100000 = 5.9  km/h
    float ERPM_current3 = 8644.0f; // ((8644/6)/5.7) * 63 * 3.14 * 60 / 100000 = 30km/h

    if (erpm <= ERPM_current1) {
        return 100.0f;
    }

    if (erpm <= ERPM_current2) {
        // Linear interpolation between 120A at 430 ERPM and 190A at 1000 ERPM
        float slope = (gear1.maxCurrent - 120.0f) / (ERPM_current2 - ERPM_current1);
        return 100.0f + slope * (erpm - ERPM_current1);
    }

    if (erpm <= ERPM_current3) {
        return gear1.maxCurrent;
    }

    if (erpm > ERPM_current3) {
        return 145.0f;
    }

    return 0.0;

    // if (erpm <= ERPM_current1) {
    //     return 120.0f;
    // } else if (erpm <= ERPM_current2) {
    //     // Linear interpolation between 120A at 430 ERPM and 190A at 1000 ERPM
    //     float slope = (gear1.maxCurrent - 120.0f) / (ERPM_current2 - ERPM_current1);
    //     return 120 + slope * (erpm - ERPM_current1);
    // } else {
    //     return gear1.maxCurrent;
    // }
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

void setVescThrottleBrake(float inputThrottle) {
    if (inputThrottle < 40) {
        float current = map_f_nochecks(inputThrottle, 0, 40, 100, 0);
        VESC.setBrakeCurrent(current);
    }

    if (inputThrottle >= 40) {
        VESC.setCurrent(map_f(inputThrottle, 40, 100, 0, maxCurrentAtERPM(VESC.data.rpm))); //PotThrottleLevelPowerLimited
    }
}

void printVescMcConfTempValues() {
    if (VESC.getMcconfTempValues()) {
        Serial.printf("l_current_min_scale: %f\n", VESC.data_mcconf.l_current_min_scale);
        Serial.printf("l_current_max_scale: %f\n", VESC.data_mcconf.l_current_max_scale);
        Serial.printf("l_min_erpm: %f\n", VESC.data_mcconf.l_min_erpm);
        Serial.printf("l_max_erpm: %f\n", VESC.data_mcconf.l_max_erpm);
        Serial.printf("l_min_duty: %f\n", VESC.data_mcconf.l_min_duty);
        Serial.printf("l_max_duty: %f\n", VESC.data_mcconf.l_max_duty);
        Serial.printf("l_watt_min: %f\n", VESC.data_mcconf.l_watt_min);
        Serial.printf("l_watt_max: %f\n", VESC.data_mcconf.l_watt_max);
        Serial.printf("l_in_current_min: %f\n", VESC.data_mcconf.l_in_current_min);
        Serial.printf("l_in_current_max: %f\n", VESC.data_mcconf.l_in_current_max);
    }
}

volatile bool getBatteryCurrent_newData = false;
void IRAM_ATTR ADCNewDataReadyISR() {
  getBatteryCurrent_newData = true;
}

uint32_t time_getBatteryCurrent = timer_u32();
void getBatteryCurrent() {
    // If we don't have new data, skip this iteration.
    if (!getBatteryCurrent_newData) {
        // while(!getBatteryCurrent_newData);
        return;
    }

    int16_t results = dedicatedADC.getLastConversionResults();

    // AUX CURRENT
    battery.current = (((float)results) / 143.636); //143.636

    _batteryCurrentUsedInElapsedTime = battery.current / (1.0f / timer_delta_s(timer_u32() - time_getBatteryCurrent));
    battery.ampHoursUsed            += _batteryCurrentUsedInElapsedTime / 3600.0;
    if (_batteryCurrentUsedInElapsedTime >= 0.0) {
        battery.ampHoursUsedLifetime    += _batteryCurrentUsedInElapsedTime / 3600.0;
    }


    _batteryWattHoursUsedUsedInElapsedTime = (battery.current * battery.voltage) / (1.0f / timer_delta_s(timer_u32() - time_getBatteryCurrent));
    battery.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;

    if ((_batteryWattHoursUsedUsedInElapsedTime >= 0.0 || BRAKING) && !battery.charging) {
        trip.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;
        estimatedRange.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;
    }


    getBatteryCurrent_newData = false;
    // Serial.printf("Time it took to diff: %f\n", timer_delta_ms(timer_u32() - time_getBatteryCurrent));

    time_getBatteryCurrent = timer_u32();
}

int buttonWait(int *buttonPressCounter, uint32_t *timeKeeper, int msToWaitBetweenInterrupts) {
    int ret = 1;
    if (timer_delta_ms(timer_u32() - *timeKeeper) >= msToWaitBetweenInterrupts) {
        *timeKeeper = timer_u32();

        // the button sends two interrupts... once when pressed, and once when released
        // so run this function once every 2 interrupts
        if (*buttonPressCounter >= 2) {
            *buttonPressCounter = 1;
            ret = 0;
        } else {
            *buttonPressCounter = *buttonPressCounter + 1;
            ret = 1;
        }
    }
    return ret;
}

bool POWER_ON = false;
void powerSwitchCallback() {
    if (digitalRead(pinPowerSwitch)) {
        POWER_ON = true;
        // setCpuFrequencyMhz(240);
    } else {
        POWER_ON = false;
        // setCpuFrequencyMhz(80);
    }
}

int buttonPressCount[4] = {2};
uint32_t timeButton[4] = {0};

int buttonsDebounceMs = 20;
void IRAM_ATTR button1Callback() { // Switch Gears
    if (buttonWait(&buttonPressCount[0], &timeButton[0], buttonsDebounceMs) == 1) {
        return;
    }
    // nothing right now
}

void IRAM_ATTR button2Callback() { // Switch Gears
    if (buttonWait(&buttonPressCount[1], &timeButton[1], buttonsDebounceMs) == 1) {
        return;
    }
    // nothing right now
}

void IRAM_ATTR button3Callback() {
    if (buttonWait(&buttonPressCount[2], &timeButton[2], buttonsDebounceMs) == 1) {
        return;
    }
    // nothing right now
}


void IRAM_ATTR button4Callback() {
    if (buttonWait(&buttonPressCount[3], &timeButton[3], buttonsDebounceMs) == 1) {
        return;
    }
    // nothing right now
}


int daysInCurrentMonth = 0;
void clock_date_and_time() {
    clockSeconds += ((float)timer_delta_us(timeCore0) / 1000000.0);
    if (clockSeconds >= 60) {
        // substract 60 seconds to "reset" the counter
        /* if we were to reset it to "0" we may lose some milliseconds
           and over time those milliseconds will become significant enough
           for the time to slowly drift away from what it should really be */
        clockSeconds -= 60;
        clockMinutes++;
    }

    if (clockMinutes >= 60) {
        clockMinutes = 0;
        clockHours++;
    }

    if (clockHours >= 24) {
        clockHours = 0;
        clockDay++;
    }

    switch (clockMonth) {
        case 1:
            daysInCurrentMonth = daysInJanuary;
            break;
        case 2:
            daysInCurrentMonth = daysInFebruary;
            break;
        case 3:
            daysInCurrentMonth = daysInMarch;
            break;
        case 4:
            daysInCurrentMonth = daysInApril;
            break;
        case 5:
            daysInCurrentMonth = daysInMay;
            break;
        case 6:
            daysInCurrentMonth = daysInJune;
            break;
        case 7:
            daysInCurrentMonth = daysInJuly;
            break;
        case 8:
            daysInCurrentMonth = daysInAugust;
            break;
        case 9:
            daysInCurrentMonth = daysInSeptember;
            break;
        case 10:
            daysInCurrentMonth = daysInOctober;
            break;
        case 11:
            daysInCurrentMonth = daysInNovember;
            break;
        case 12:
            daysInCurrentMonth = daysInDecember;
            break;
    }

    if (clockDay > daysInCurrentMonth) {
        clockDay = 1;
        clockMonth++;
    }

    if (clockMonth > 12) {
        clockMonth = 1;
        clockYear++;
    }
}

void drawLightningBolt(int x, int y) {
    tft.drawLine(x,   y,   x+3, y+3, COLOR_WHITE);
    tft.drawLine(x+1, y+4, x+2, y+4, COLOR_WHITE);
    tft.drawLine(x,   y+5, x+4, y+9, COLOR_WHITE);

    tft.drawLine(x+3, y+9, x+1, y+9, COLOR_WHITE);
    tft.drawLine(x+4, y+8, x+4, y+6, COLOR_WHITE);
}

void printDisplay() {
    tft.clearDisplay();

    tft.setCursor(0, 0); tft.printf ("V: %4.1f ", battery.voltage);
    tft.setCursor(0, 0+9); tft.printf ("R:%3.0f ", estimatedRange.range);
    tft.setCursor(0, 0+18); tft.printf ("U:%3.0f ", wh_over_km_average);

    tft.setCursor(SCREEN_WIDTH - (6*5), 0); tft.printf ("%4.0f%%", battery.percentage);

    if (battery.charging)
        drawLightningBolt(SCREEN_WIDTH - 10, 10);

    // Speed
    static int SPEED_HEIGHT = 18;
    int digitsSpace = speed_kmh < 10 ? 5 : 11;
    tft.setTextSize(3);
    tft.setCursor((SCREEN_WIDTH / 2) - (digitsSpace / 2), SPEED_HEIGHT); tft.printf("%1.0f", speed_kmh);
    tft.setTextSize(1);

    // Power profile
    int len = strlen(VESC.data_mcconf.name.c_str());
    tft.setCursor(SCREEN_WIDTH / 2 - ((len*5)/2), SPEED_HEIGHT + 24); tft.printf("%s", VESC.data_mcconf.name.c_str());

    // Bottom Divider
    static int BOTTOM_DIVIDER_Y = 55;
    tft.drawLine(0, BOTTOM_DIVIDER_Y, SCREEN_WIDTH, BOTTOM_DIVIDER_Y, COLOR_WHITE);

    // Odometer text
    tft.setCursor(0, BOTTOM_DIVIDER_Y-9);
    tft.printf("odo");

    // Odometer distance
    tft.setCursor(0, BOTTOM_DIVIDER_Y+2);
    tft.printf("%.0f", odometer.distance);

    // Trip text
    tft.setCursor(SCREEN_WIDTH-24, BOTTOM_DIVIDER_Y-9);
    tft.printf("trip");

    // Trip distance
    tft.setCursor(SCREEN_WIDTH-30, BOTTOM_DIVIDER_Y+2);
    tft.printf("%5.1f", trip.distance);

    tft.display();
}

// runs on core 1
void loop_core1 (void* pvParameters) {
    while (1) {
        timeStartCore1 = timer_u32();

        // Reset temporary values
        _batteryVoltage = 0;
        _potThrottleVoltage = 0;

        // measure raw battery and pot voltage from ADC
        for (int i = 0; i < 15; i++) {
            _batteryVoltage += adc1_get_raw(pinBatteryVoltage);
            _potThrottleVoltage += adc1_get_raw(pinPotThrottle);
        }
        _batteryVoltage = _batteryVoltage / 15;
        _potThrottleVoltage = _potThrottleVoltage / 15;

        // BATTERY VOLTAGE
        battery.voltage = (36.11344125582536f * BatVoltageCorrection.correctInput(_batteryVoltage * (3.3f/4095.0f)));
        BatVoltageMovingAverage.initInput(battery.voltage);

        // BATTERY CURRENT
        // Calculates wattHoursUsed, ampHoursUsed, ampHoursUsedLifetime
        getBatteryCurrent();

        battery.watts = battery.voltage * battery.current;
        battery.wattHoursRated = battery.nominalVoltage * battery.ampHoursRated;

        // BRAKING = digitalRead(pinBrake);
        // TODO: feature idea: when we manually start to roll the bike from a standstill, dont activate braking until some throttle is applied
        if (speed_kmh > 1.0 && PotThrottleLevelReal == 0) {
            BRAKING = true;
        } else {
            BRAKING = false;
        }

        // automatically change the battery.charging status to true if its false & we aint braking & the current is less than 0 (meaning it's charging)
        // we don't have dedicated sensing pin for the charger being connected, yet...
        // after that wait one second, just to make sure it's not a fluke, so it doesnt just randomly put it in charging mode because of a small anomaly...
        if (!battery.charging && !BRAKING && battery.current < 0.0) {
            if (timer_delta_ms(timer_u32() - timeWaitBeforeChangingToCharging) >= 1000) {
                battery.charging = true;
            }
        } else {
            timeWaitBeforeChangingToCharging = timer_u32();
        }

        // Battery charge tracking stuff
        if (settings.batteryPercentageVoltageBased) {
            battery.percentage = map_f(battery.voltage, battery.voltage_min, battery.voltage_max, 0, 100);
        } else {
            battery.percentage = map_f_nochecks(battery.ampHoursUsed, 0.0, battery.ampHoursRated, 100.0, 0.0);
        }

        BatVoltageMovingAverage.moveAverage(battery.voltage);
        if (!battery.charging && (BatVoltageMovingAverage.output <= battery.amphours_min_voltage) && (battery.current <= 3.0 && battery.current >= 0.0)) {
            if (timer_delta_ms(timer_u32() - timeAmphoursMinVoltage) >= 5000) {
                battery.ampHoursRated_tmp = battery.ampHoursUsed; // save ampHourUsed to ampHourRated_tmp...
                                                                // this temporary value will later get applied to the actual
                                                                // ampHourRated variable when the battery is done charging so
                                                                // the estimated range and other stuff doesn't suddenly get screwed
            }
        } else {
            timeAmphoursMinVoltage = timer_u32();
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

        // Map throttle
        potThrottleVoltage = (
            PotThrottleCorrection.correctInput(
                PotThrottleMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _potThrottleVoltage
                )
            )
        );
        PotThrottleLevelReal = map_f(potThrottleVoltage, 0.9, 2.4, 0, 100);

        // Throttle
        if (!POWER_ON || battery.charging) {
            VESC.setCurrent(0.0f);
        } else if (BRAKING && settings.regenerativeBraking) {
            static float brakingCurrentMax = 100.0;

            // gradually increase braking current
            VESC.setBrakeCurrent(movingAverages.brakingCurrent.moveAverage(brakingCurrentMax));
        } else {
            VESC.setCurrent(throttle.map(PotThrottleLevelReal));

            // reset regen braking
            movingAverages.brakingCurrent.setInput(0.0f);
        }

        static uint32_t timeAcceleration;
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
            speed_kmhMovingAverage.moveAverage(speed_kmh);
            // if (timer_delta_ms(timer_u32() - timeAcceleration) >= 100) {
                acceleration = (speed_kmhMovingAverage.output - speed_kmh_previous) * (1.0 / timer_delta_s(timer_u32() - timeAcceleration));

                timeAcceleration = timer_u32();
                speed_kmh_previous = speed_kmhMovingAverage.output;
            // }

        }

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

        // Execute every second that elapsed
        if (timer_delta_ms(timer_u32() - timeExecEverySecondCore1) >= 1000) {
            timeExecEverySecondCore1 = timer_u32();
            // stuff to run

        }

        // Execute every 100ms that elapsed
        if (timer_delta_ms(timer_u32() - timeExecEvery100millisecondsCore1) >= 10000) {
            timeExecEvery100millisecondsCore1 = timer_u32();

        }
        timeCore1 = (timer_u32() - timeStartCore1);
    }
}

// runs on core 0
void app_main(void)
{
    motor.poles = 18;
    motor.magnetPairs = 3;
    wheel.diameter = 63.0f;
    wheel.gear_ratio = 10.6875f; // (42/14) * (57/16)
    battery.voltage_min = 66.0f;
    battery.voltage_max = 82.0f;
    battery.nominalVoltage = 72.0f;

    battery.amphours_min_voltage = 68.0f;
    battery.amphours_max_voltage = 82.0f;
    battery.ampHoursRated_tmp = 0.0;

    // GEARS
    gear1.level = 1;
    gear1.maxCurrent = 200; // 160

    gear2.level = 2;
    gear2.maxCurrent = 100;

    gear3.level = 3;
    gear3.maxCurrent = 65;

    gearCurrent = gear1;

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

    BatVoltageCorrection.offsetPoints = { // 12DB Attenuation... can read up to 3.3V
    // input, offset
    // TODO: redo these measurements
    {0.0, 0},
    {0.150, 0.098},
    {0.200, 0.095},
    {0.500, 0.095},
    {0.550, 0.09},
    {0.600, 0.092},
    {0.650, 0.092},
    {0.700, 0.09},
    {0.750, 0.09},
    {0.800, 0.09},
    {0.850, 0.09},
    {0.900, 0.088},
    {0.950, 0.085},
    {1.000, 0.082},
    {1.100, 0.08},
    {1.200, 0.077},
    {1.300, 0.071},
    {1.400, 0.068},
    {1.500, 0.068},
    {1.600, 0.068},
    {1.700, 0.066},
    {1.800, 0.062},
    {1.900, 0.062},
    {2.000, 0.059},
    {2.100, 0.06},
    {2.200, 0.055},
    {2.300, 0.053},
    {2.400, 0.045},
    {2.500, 0.038},
    {2.600, 0.02},
    {2.700, -0.005},
    {2.800, -0.048},
    {2.900, -0.105},
    {3.000, -0.183},
    {3.050, -0.225}
    };
    PotThrottleCorrection.offsetPoints = BatVoltageCorrection.offsetPoints;

    // smoothing factors
    BatVoltageMovingAverage.smoothingFactor = 0.1; //0.2
    BatWattMovingAverage.smoothingFactor = 0.1;
    PotThrottleMovingAverage.smoothingFactor = 0.5; // 0.5
    WhOverKmMovingAverage.smoothingFactor = 0.05;
    batteryAuxWattsMovingAverage.smoothingFactor = 0.2;
    batteryAuxMovingAverage.smoothingFactor = 0.2;
    batteryWattsMovingAverage.smoothingFactor = 0.2;
    speed_kmhMovingAverage.smoothingFactor = 0.02;
    wattageMoreSmooth_MovingAverage.smoothingFactor = 0.4f;
    movingAverages.brakingCurrent.smoothingFactor = 0.025f;

    initArduino();
    Serial.begin(230400);

    Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX/TX
    VESC.setSerialPort(&Serial2);
    // VESC.setDebugPort(&Serial);

    Wire.begin(21, 22);
    Wire.setClock(400000); // 400kHz

    if (!dedicatedADC.begin()) {
        Serial.println("Failed to initialize ADS.");
    }
    // Up-to 64SPS is noise-free, 128SPS has a little noise, 250 and beyond is noise af
    dedicatedADC.setDataRate(RATE_ADS1115_64SPS);
    dedicatedADC.setGain(GAIN_SIXTEEN);
    pinMode(pinAdcRdyAlert, INPUT);
    // We get a falling edge every time a new sample is ready.
    attachInterrupt(pinAdcRdyAlert, ADCNewDataReadyISR, FALLING);
    dedicatedADC.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);


    Wire1.begin(25, 26);
    Wire1.setClock(400000);
    if(!tft.begin(SCREEN_ADDRESS, true)) {
        Serial.println(F("SH1106 allocation failed"));
    }
    tft.clearDisplay();
    tft.setTextColor(SH110X_WHITE, SH110X_BLACK);

    // Configure ADC width (resolution)
    adc1_config_width(ADC_WIDTH);
    // Configure ADC channel and attenuation
    adc1_config_channel_atten(pinBatteryVoltage, ADC_ATTEN); // PIN 36/VP
    adc1_config_channel_atten(pinPotThrottle,    ADC_ATTEN); // PIN 34

    // Configure buttons
    pinMode(pinButton1, INPUT_PULLDOWN);
    pinMode(pinButton2, INPUT_PULLDOWN);
    pinMode(pinButton3, INPUT_PULLDOWN);
    pinMode(pinButton4, INPUT_PULLDOWN);
    attachInterrupt(pinButton1, button1Callback, GPIO_INTR_POSEDGE);
    attachInterrupt(pinButton2, button2Callback, GPIO_INTR_POSEDGE);
    attachInterrupt(pinButton3, button3Callback, GPIO_INTR_POSEDGE);
    attachInterrupt(pinButton4, button4Callback, GPIO_INTR_POSEDGE);

    // Power Switch
    pinMode(pinPowerSwitch, INPUT_PULLDOWN);
    attachInterrupt(pinPowerSwitch, powerSwitchCallback, GPIO_INTR_ANYEDGE);
    powerSwitchCallback();

    // Braking Switch
    pinMode(pinBrake, INPUT_PULLDOWN);

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = static_cast<ledc_timer_bit_t>(8),
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000, //1000
        .clk_cfg = LEDC_USE_APB_CLK,
        .deconfigure = false
    };
    ledc_timer_config(&timer_conf);

    // Channel Configuration
    ledc_channel_config_t channel_conf = {
        .gpio_num = pinRotor,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {
            .output_invert = 0
        }
    };
    ledc_channel_config(&channel_conf);

    for (int i = 0; i < 4; i++){
        buttonPressCount[i] = 2;
        timeButton[i] = 0;
    }

    powerLimiterPID.setOutputLimits(-100, 0);
    // powerLimiterPID.setSetpoint(50); // max power watts

    // setup ebike namespace
    preferences.begin("ebike", false);

    // retrieve values      
    odometer.distance       = preferences.getFloat("odometer", -1);
    battery.ampHoursUsed    = preferences.getFloat("batAhUsed", -1);
    battery.ampHoursRated   = preferences.getFloat("batAhFulDis", -1);
    battery.ampHoursUsedLifetime = preferences.getFloat("batAhUsedLife", 0);
    settings.regenerativeBraking = preferences.getBool("regenBrake", 0);

    estimatedRangeReset();
    odometer.distance_tmp = odometer.distance;

    xTaskCreatePinnedToCore (
        loop_core1,     // Function to implement the task
        "loop_core1",   // Name of the task
        10000,      // Stack size in bytes
        NULL,      // Task input parameter
        0,         // Priority of the task
        NULL,      // Task handle.
        1          // Core where the task should run
    );

    while(1) {
        timeStartCore0 = timer_u32();

        // Read and store all characters into a string
        while (Serial.available()) {
          char c = Serial.read();
          readString += c;
        }

        static std::string toSend;
        if (!readString.empty()) {
            auto readStringPacket = split(readString, '\n');

            for (int i = 0; i < readStringPacket.size(); i++) {
                auto packet = split(readStringPacket[i], ';');
                int packet_command_id = std::stoi(packet[0]);

                toSend = "";
                switch(packet_command_id) {
                    case COMMAND_ID::GET_BATTERY:
                        commAddValue(&toSend, COMMAND_ID::GET_BATTERY, 0);
                        commAddValue(&toSend, battery.voltage, 1);
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

                    case COMMAND_ID::GET_STATS:
                        commAddValue(&toSend, COMMAND_ID::GET_STATS, 0);
                        commAddValue(&toSend, speed_kmh, 1);
                        commAddValue(&toSend, motor_rpm, 0);
                        commAddValue(&toSend, odometer.distance, 1);
                        commAddValue(&toSend, trip.distance, 2);
                        commAddValue(&toSend, gearCurrent.level, 0);
                        commAddValue(&toSend, gearCurrent.maxCurrent, 0);
                        commAddValue(&toSend, -1, 0); // was selectedPowerMode
                        commAddValue(&toSend, VESC.data.avgMotorCurrent, 1);
                        commAddValue(&toSend, wh_over_km_average, 1);
                        commAddValue(&toSend, estimatedRange.WhPerKm, 1);
                        commAddValue(&toSend, estimatedRange.range, 1);
                        commAddValue(&toSend, VESC.data.tempMotor, 1);
                        commAddValue(&toSend, totalSecondsSinceBoot, 0);
                        commAddValue(&toSend, timer_delta_us(timeCore0), 1);
                        commAddValue(&toSend, timer_delta_us(timeCore1), 1);
                        commAddValue(&toSend, acceleration, 1);
                        commAddValue(&toSend, POWER_ON, 0);
                        commAddValue(&toSend, settings.regenerativeBraking, 0);
                   
                        toSend.append("\n");
                        break;

                    case COMMAND_ID::SET_ODOMETER:
                        odometer.distance = (float)getValueFromPacket(packet, 1);
                        toSend.append(std::format("{};Odometer was set to: {} km;\n", static_cast<int>(COMMAND_ID::ESP32_LOG), odometer.distance));
                        break;

                    case COMMAND_ID::SAVE_PREFERENCES:
                        preferenceActions.saveAll();
                        toSend.append(std::format("{};Preferences were manually saved;\n", static_cast<int>(COMMAND_ID::ESP32_LOG)));
                        break;
                        
                    case COMMAND_ID::RESET_TRIP:
                        tripReset();
                        toSend.append(std::format("{};Trip was reset;\n", static_cast<int>(COMMAND_ID::ESP32_LOG)));
                        break;

                    case COMMAND_ID::RESET_ESTIMATED_RANGE:
                        estimatedRangeReset();
                        toSend.append(std::format("{};Estimated range was reset;\n", static_cast<int>(COMMAND_ID::ESP32_LOG)));
                        break;

                    case COMMAND_ID::READY_TO_WRITE:
                        commAddValue(&toSend, COMMAND_ID::READY_TO_WRITE, 0);

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::GET_FW:
                        commAddValue(&toSend, COMMAND_ID::GET_FW, 0);
                        toSend.append(std::format("{};{}; {} {};", EBIKE_NAME, EBIKE_VERSION, __DATE__, __TIME__)); // NAME, VERSION, COMPILE DATE/TIME

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::PING:
                        display.ping();
                        break;

                    case COMMAND_ID::TOGGLE_FRONT_LIGHT:

                        break;

                    case COMMAND_ID::SET_AMPHOURS_USED_LIFETIME:
                        battery.ampHoursUsedLifetime = (float)getValueFromPacket(packet, 1);
                        toSend.append(std::format("{};Amphours used (Lifetime) was set to: {} Ah;\n", static_cast<int>(COMMAND_ID::ESP32_LOG), battery.ampHoursUsedLifetime));
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

                        toSend.append(std::format("{};VESC McConf was sent;\n", static_cast<int>(COMMAND_ID::ESP32_LOG)));
                        break;

                    case COMMAND_ID::SET_AMPHOURS_CHARGED:
                        {
                            float newValue = getValueFromPacket(packet, 1);

                            battery.ampHoursRated = newValue;
                            battery.ampHoursRated_tmp = newValue;

                            toSend.append(std::format("{};Amphours charged was set to: {} Ah;\n", static_cast<int>(COMMAND_ID::ESP32_LOG), newValue));
                        }
                        break;

                    case COMMAND_ID::TOGGLE_CHARGING_STATE:
                        if (battery.charging) {
                            battery.charging = false;
                        } else {
                            battery.charging = true;
                        }

                        toSend.append(std::format("{};Charging state was toggled, now set to: {};\n", static_cast<int>(COMMAND_ID::ESP32_LOG), battery.charging));
                        break;

                    case COMMAND_ID::TOGGLE_REGEN_BRAKING:
                        if (settings.regenerativeBraking) {
                            settings.regenerativeBraking = false;
                        } else {
                            settings.regenerativeBraking = true;
                        }

                        toSend.append(std::format("{};Regenerative braking state was toggled, now set to: {};\n", static_cast<int>(COMMAND_ID::ESP32_LOG), settings.regenerativeBraking));
                        break;

                    case COMMAND_ID::ESP32_RESTART:
                        ESP.restart();
                        break;
                }

                if (strlen(toSend.c_str()) > 0) {
                    Serial.printf(toSend.c_str());
                }
            }

            readString="";
        }

        // totalSecondsSinceBoot += ((float)timer_delta_us(timeCore0) / 1000000);
        clockSecondsSinceBoot = (int)(totalSecondsSinceBoot) % 60;
        clockMinutesSinceBoot = (int)(totalSecondsSinceBoot / 60) % 60;
        clockHoursSinceBoot   = (int)(totalSecondsSinceBoot / 60 / 60) % 24;
        clockDaysSinceBoot    = totalSecondsSinceBoot / 60 / 60 / 24;

        // function for counting the current time and date
        clock_date_and_time();

        static bool displayPreviousOnState = true;

        if (!display.isExternalDisplayConnected()) { //  && POWER_ON
            printDisplay();
            displayPreviousOnState = true;
        } else if (displayPreviousOnState == true) {
            displayPreviousOnState = false;

            tft.clearDisplay();
            tft.display();
        }

        // Execute every second that elapsed
        timeStartExecEverySecondCore0 = timer_u32();
        // float millisecondsInTicks = (1000.0/*ms*/ / (0.001 * _TICKS_PER_US));
        if (timer_delta_ms(timeStartExecEverySecondCore0 - timeExecEverySecondCore0) >= 1000.0) {

            // totalSecondsSinceBoot_uint64++;
            totalSecondsSinceBoot += timer_delta_s(timeStartExecEverySecondCore0 - timeExecEverySecondCore0);

            if (settings.printCoreExecutionTime) {
                Serial.printf("\n\rExecution time:"
                            "\n\r core0: %.1f us   "
                            "\n\r core1: %.1f us   "
                            "\n\r timer_u32(): %llu ns                      \n\r",
                            timer_delta_us(timeCore0), timer_delta_us(timeCore1), timer_u32());
            }

            timeExecEverySecondCore0 = timeStartExecEverySecondCore0;
        }

        unsigned long timeSavePreferencesNow = millis();
        unsigned long timeSavePreferencesElapsed = timeSavePreferencesNow - timeSavePreferencesStart;
        if (timeSavePreferencesElapsed >= (10 * 60 * 1000)) { //10 minutes
            timeSavePreferencesStart = millis();

            preferenceActions.saveBattery();
            preferenceActions.saveOther();

            if (odometer.distance_tmp != odometer.distance) {
                odometer.distance_tmp = odometer.distance;

                preferenceActions.saveOdometer();
            }
            toSendExtra.append(std::format("{};Preferences saved;\n", static_cast<int>(COMMAND_ID::ESP32_LOG)));
        }

        if (strlen(toSendExtra.c_str()) > 0) {
            Serial.printf(toSendExtra.c_str());
        }
        
        timeCore0 = (timer_u32() - timeStartCore0);
    }
}
