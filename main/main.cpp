#include <stdio.h>
#include <iostream>
#include <chrono>
#include <mutex>
#include <print>
#include <utility>

// Arduino Libraries
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "HardwareSerial.h"
#include "Preferences.h"
#include <Adafruit_ADS1X15.h>
#include <VescUart.h>
#include <TFT_eSPI.h>

#include "esp32-hal-dac.h"
#include "esp32-hal-ledc.h"
#include "hal/dac_types.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"

#include "inputOffset.h"
#include "fastGPIO.h"
#include "ebike-utils.h"
#include "timer_u32.h"
#include "MiniPID.h"
#include "map.cpp"

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
};

struct {
    bool automaticGearChanging = 1;
    bool drawDebug = 0;
    bool drawUptime = 1;
    bool printCoreExecutionTime = 0;
    bool disableOptimizedDrawing = 0;
    bool batteryPercentageVoltageBased = 0;
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
    double DNU_refresh;
    double wattHoursUsed;
} trip;

struct {
    double trip_distance;    // in km
    double distance;         // in km
    double distance_tmp;
    double DNU_refresh;
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
} battery;

struct {
    int level;
    int maxCurrent;
} gear1, gear2, gear3, gearCurrent;

struct {
    float range;
    float ampHoursUsedOnStart;
    float tripOnStart;
    float distance;
    float ampHoursUsed;
    float AhPerKm;
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

uint32_t timeStartCore1 = 0, timeCore1 = 0;    
uint32_t timeStartCore0 = 0, timeCore0 = 0;    
uint32_t timeStartDisplay = 0; //timeDisplay = 0;
uint32_t timeStartExecEverySecondCore0 = 0, timeExecEverySecondCore0 = 0;
uint32_t timeExecEverySecondCore1 = 0;
uint32_t timeExecEvery100millisecondsCore1 = 0;
uint32_t timeRotorSleep = 0;
uint32_t timeAmphoursMinVoltage = 0;
unsigned long timeSavePreferencesStart = millis();

bool rotorCanPowerMotor = 1;
bool rotorCutOff_temp = 1;

// uptime
float totalSecondsSinceBoot;
int clockSecondsSinceBoot, DNU_clockSecondsSinceBoot;
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
float clockSeconds = 15, DNU_clockSeconds;
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

// settings for gearing and power
int selectedGear, DNU_selectedGear;
int selectedPowerMode, DNU_selectedPowerMode;

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

ThrottleMap throttle;

Display display;

Preferences preferences;

TFT_eSPI tft = TFT_eSPI();
Adafruit_ADS1115 dedicatedADC;
VescUart VESC;

double kP = 1, kI = 0.02, kD = 0.5; //kP = 0.1, 0.3 is unstable
MiniPID powerLimiterPID(kP, kI, kD);

#define pinRotor          25                // D25
#define pinBatteryVoltage ADC1_CHANNEL_0    // D36
#define pinPotThrottle    ADC1_CHANNEL_3    // D39 // VN
#define pinTFTbacklight   2                 // D2
#define pinButton1  -1  // D4
#define pinButton2  -1 // RX2
#define pinButton3  -1 // TX2
#define pinButton4  -1 // D21 // Temporarily set to D26 for I2C
#define pinAdcRdyAlert 33 // D33
#define pinPowerSwitch 27 // D27

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
    estimatedRange.ampHoursUsedOnStart  = battery.ampHoursUsed;
    estimatedRange.distance             = 0.0;
    estimatedRange.ampHoursUsed         = 0.0;
}

void tripReset() {
    trip.distance = 0;
    trip.wattHoursUsed = 0;
    // estimatedRangeReset();
}

void preferencesSaveOdometer() {
    preferences.putFloat("odometer", (float)odometer.distance);
}

void preferencesSaveBattery() {
    preferences.putFloat("batAhUsed", battery.ampHoursUsed);
    preferences.putFloat("batAhFulDis", battery.ampHoursRated);
    preferences.putFloat("batAhUsedLife", battery.ampHoursUsedLifetime);
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
        estimatedRange.ampHoursUsed += _batteryCurrentUsedInElapsedTime / 3600.0;
    }

    _batteryWattHoursUsedUsedInElapsedTime = (battery.current * battery.voltage) / (1.0f / timer_delta_s(timer_u32() - time_getBatteryCurrent));
    if (_batteryWattHoursUsedUsedInElapsedTime >= 0.0) {
            battery.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;

            // TODO: later move it outside of this if function, and only allow subtracting from the wattHoursUsed variable
            //       when using regenerative braking
            trip.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;
    }


    getBatteryCurrent_newData = false;
    // Serial.printf("Time it took to diff: %f\n", timer_delta_ms(timer_u32() - time_getBatteryCurrent));

    time_getBatteryCurrent = timer_u32();
}

int PowerLevelnegative1Watts = 1000000;
int PowerLevel0Watts = 100;
int PowerLevel1Watts = 250;
int PowerLevel2Watts = 500;
int PowerLevel3Watts = 1000;

void setPowerLevel(int level) {
    if (level == -1) {
        // powerLimiterPID.setSetpoint(PowerLevelnegative1Watts);
        selectedPowerMode = -1;
    }

    if (level == 0) {
        // powerLimiterPID.setSetpoint(PowerLevel0Watts);
        selectedPowerMode = 0;
    }

    if (level == 1) {
        // powerLimiterPID.setSetpoint(PowerLevel1Watts);
        selectedPowerMode = 1;
    }

    if (level == 2) {
        // powerLimiterPID.setSetpoint(PowerLevel2Watts);
        selectedPowerMode = 2;
    }

    if (level == 3) {
        // powerLimiterPID.setSetpoint(PowerLevel3Watts);
        selectedPowerMode = 3;
    }
}

// rotor electromagnet DC resistance is around 3,05 ohms
int GearLevel0DutyCycle = 0; // 0W
int GearLevel1DutyCycle = 255; // ~180W  // 225
int GearLevel2DutyCycle = 255; // ~100W //180
int GearLevel3DutyCycle = 110; // ~50W //110
int GearLevelTrackSpeedDutyCycle = 70; // There is some power so the VESC can track the speed
int GearLevelIdleLittleCurrentDutyCycle = 20; //10 // There is some power so the VESC can track the speed
int GearDutyCycle;

void setGearLevel(int level) {
    if (level == 0) {
        selectedGear = 0;
        GearDutyCycle = GearLevel0DutyCycle;
        // gearCurrent = gear1;
    }

    if (level == 1) {
        selectedGear = 1;
        GearDutyCycle = GearLevel1DutyCycle;
        gearCurrent = gear1;
    }

    if (level == 2) {
        selectedGear = 2;
        GearDutyCycle = GearLevel2DutyCycle;
        gearCurrent = gear2;
    }

    if (level == 3) {
        selectedGear = 3;
        GearDutyCycle = GearLevel3DutyCycle;
        gearCurrent = gear3;
    }

    setDuty(LEDC_CHANNEL_0, GearDutyCycle);
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

    switch(selectedGear) {
        case 3:
            setGearLevel(2);
            break;
        case 2:
            setGearLevel(1);
            break;
        case 1:
            setGearLevel(0);
            break;
        default:
            break;
    }
}

void IRAM_ATTR button2Callback() { // Switch Gears
    if (buttonWait(&buttonPressCount[1], &timeButton[1], buttonsDebounceMs) == 1) {
        return;
    }

    switch(selectedGear) {
        case 0:
            setGearLevel(1);
            break;
        case 1:
            setGearLevel(2);
            break;
        case 2:
            setGearLevel(3);
            break;
        default:
            break;
    }
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

class VerticalBar {
private:
    int previousValue = -1;

public:
    void drawVerticalBar(int x, int y, int width, int height, float value, float value_min, float value_max, char *barName, bool drawInputValue) {
        int lastValue = -1;
        int value_map = map_f(value, value_min, value_max, 0.0, 100.0);

        if ((int)value == lastValue)
            return; // No need to redraw

        lastValue = value_map;

        int filledHeight = map(value_map, 0, 100, 0, height);
        int greenEnd  = map(60, 0, 100, 0, height);
        int yellowEnd = map(85, 0, 100, 0, height);

        int baseY = y + height;

        tft.setTextSize(2);
        tft.setCursor(x+3, y-16);
        tft.printf("%s", barName);
        tft.setTextSize(1);
        tft.setCursor(x+3, baseY+2);
        if (drawInputValue == true)
            tft.printf("%1.0f  ", value);
        else
            tft.printf("%1d  ", value_map);

        // Draw filled portion

        // inside of border
        // tft.fillRect(x+1, y+1, width-2, height-2, TFT_DARKGREY);

        // only refill a part of the rectangle so that it doesn't flicker by clearing the whole rectangle and then again drawing over it
        tft.fillRect(x+1, y+1, width-2, height - filledHeight, TFT_LIGHTGREY);

        // // Red zone
        // if (filledHeight > yellowEnd)
        //     tft.fillRect(x, baseY - filledHeight, width, filledHeight - yellowEnd, TFT_RED);

        // // Yellow zone
        // if (filledHeight > greenEnd)
        //     tft.fillRect(x, baseY - min(filledHeight, yellowEnd), width, min(filledHeight, yellowEnd) - greenEnd, TFT_YELLOW);

        // // Green zone
        // if (filledHeight > 0)
        //     tft.fillRect(x, baseY - min(filledHeight, greenEnd), width, min(filledHeight, greenEnd), TFT_GREEN);

        // Red zone all the way
        if (filledHeight > 0)
            tft.fillRect(x+1, baseY - filledHeight + 1, width-2, filledHeight -2, TFT_RED);

        // Border
        tft.drawRect(x, y, width, height, TFT_BLACK);
    }
};
VerticalBar barAp;
VerticalBar barW;

void redrawScreen() {
    tft.fillScreen(TFT_WHITE);
    firsttime_draw = 1;
}

void printDisplay() {
    // TODO: only update the variables, not the whole text, as it takes longer to draw

    // TODO: Disable clock until time synchronization is implemented
    // // Clock
    // tft.setTextSize(2);
    // if (firsttime_draw || ((int)DNU_clockSeconds != (int)clockSeconds)) {
    //     DNU_clockSeconds = clockSeconds;
    //     tft.setCursor(3, 3); tft.printf("%02d:%02d:%02.0f", clockHours, clockMinutes, clockSeconds);
    //     tft.setTextSize(1);
    //     tft.setCursor(100, 10); tft.printf("%1d.%1d.%4d  ", clockDay, clockMonth, clockYear);
    // } 
    tft.setTextSize(2);

    // Battery
    tft.setCursor(236, 3); tft.printf("%6.1f%%", battery.percentage);
    // Battery with amphours used
    // tft.setCursor(150, 3); tft.printf("%5.3fAh %4.1f%%", battery.ampHoursUsed, battery.percentage); //batteryAmpHours

    // Upper Divider
    if (firsttime_draw) {
        tft.drawLine(0, 20, 320, 20, TFT_BLACK);
    }

    // Power Draw // Battery Voltage // Battery Amps draw
    tft.setCursor(3, 28); tft.printf ("W:%6.1f  ", batteryWattsMovingAverage.moveAverage(battery.watts));
    tft.setCursor(3, 47); tft.printf ("V:  %4.1f ", battery.voltage);
    tft.setCursor(3, 66); tft.printf ("A:  %6.3f ", batteryAuxMovingAverage.moveAverage(battery.current));
    tft.setCursor(3, 85); tft.printf ("Ah: %6.3f ", battery.ampHoursUsed);
    tft.setCursor(3, 104); tft.printf("T: %3.0f C ", VESC.data.tempMotor);
    // tft.setCursor(3, 104); tft.printf("R: %3.1fkm ", estimatedRange.range);
    // tft.setCursor(3, 85); tft.printf("vA: %6.3f vAp: %5.1f  ", VESC.data.avgInputCurrent, VESC.data.avgMotorCurrent); //vescCurrent, vescAmpHours
    // tft.setCursor(3, 85); tft.printf("vA: %6.3f ", VESC.data.avgInputCurrent); //vescCurrent, vescAmpHours
    // tft.setCursor(3, 104); tft.printf("vAh: %5.3f", VESC.data.ampHours);
    tft.setCursor(3, 123); tft.printf("Pot:%3.0f%%", PotThrottleLevelReal);
    // tft.setCursor(3, 142); tft.printf("Dut:%3.0f%%", VESC.data.dutyCycleNow);

    // tft.setCursor(3, 104); tft.printf("A: %6.3f  Ah: %5.4f  ", auxCurrent, auxAmpHours);
    // tft.setCursor(3, 85); tft.printf("A: %6.3f  ", vescCurrent);
    // tft.setCursor(3, 142); tft.printf("Pl: %3.0f%% ", PotThrottleLevelPowerLimited);
    // tft.setCursor(3, 124); tft.printf("Pot:%5.2f", potThrottle_voltage);

    // consumption over last 1km
    tft.setCursor(207-20, 28); tft.printf("%5.1f Wh/km", WhOverKmMovingAverage.moveAverage(wh_over_km));
    tft.setCursor(207-20, 28+19); tft.printf("%5.1f Wh/km", wh_over_km_average);
    tft.setCursor(207-20, 28+19+19); tft.printf("%5.1f km", estimatedRange.range);

    // Speed
    tft.setTextSize(5);
    tft.setCursor(100, 106); tft.printf("%4.1f", speed_kmh);
    tft.setCursor(248, 127); tft.setTextSize(2);
    // if (firsttime_draw) {
    //     tft.println("km/h");
    // }

    // Gear // Power Level
    if (firsttime_draw || (DNU_selectedGear != selectedGear) || (DNU_selectedPowerMode != selectedPowerMode)) {
        DNU_selectedGear = selectedGear;
        DNU_selectedPowerMode = selectedPowerMode;

        // tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
        tft.setCursor(125, 147); tft.printf("Gear:  %d", gearCurrent.level);
        tft.setCursor(125, 166); tft.printf("Power: %d", selectedPowerMode);
        // tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    }

    // Uptime
    tft.setTextSize(1);
    if (settings.drawUptime && (firsttime_draw || DNU_clockSecondsSinceBoot != clockSecondsSinceBoot))  {
        DNU_clockSecondsSinceBoot = clockSecondsSinceBoot;
        tft.setCursor(3, 205);
        tft.printf("Uptime: %2dd %2dh %2dm %2ds\n", clockDaysSinceBoot, clockHoursSinceBoot, clockMinutesSinceBoot, clockSecondsSinceBoot);
    }

    if (firsttime_draw) {
        // Bottom Divider
        tft.drawLine(0, 215, 320, 215, TFT_BLACK);
    }

    // Odometer
    if (firsttime_draw || odometer.DNU_refresh != odometer.distance) {
        odometer.DNU_refresh = odometer.distance;
        tft.setTextSize(2);
        tft.setCursor(3, 220);
        tft.printf("O: %.0f", odometer.distance);
    }

    // Trip
    if (firsttime_draw || trip.DNU_refresh != trip.distance) {
        trip.DNU_refresh = trip.distance;
        tft.setTextSize(2);
        tft.setCursor(220, 220);
        tft.printf("T: %.2f", trip.distance);
    }

    if (settings.drawDebug) {
        tft.setTextSize(1);
        sprintf(text, "\nExecution time:"
                        "\n core0: %.1f us   "
                        "\n core1: %.1f us   \n",
                        timer_delta_us(timeCore0), timer_delta_us(timeCore1));
        tft.setCursor(160, 150);
        tft.println(text);
    }

    barAp.drawVerticalBar(280, 104, 30, 92, VESC.data.avgMotorCurrent, 0, gear1.maxCurrent, "Ap", true);
    barW.drawVerticalBar(240, 104, 30, 92, VESC.data.dutyCycleNow, 0.0, 1.0, "D", false);
    // barW.drawVerticalBar(240, 104, 30, 92, battery.watts, 0, 5000, "W");

    if (firsttime_draw) {
        firsttime_draw = settings.disableOptimizedDrawing; // should be 0
    }
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

        // Battery charge tracking stuff
        if (settings.batteryPercentageVoltageBased) {
            battery.percentage = map_f(battery.voltage, battery.voltage_min, battery.voltage_max, 0, 100);
        } else {
            battery.percentage = map_f_nochecks(battery.ampHoursUsed, 0.0, battery.ampHoursRated, 100.0, 0.0);
        }

        BatVoltageMovingAverage.moveAverage(battery.voltage);
        if (BatVoltageMovingAverage.output <= battery.amphours_min_voltage) {
            if (timer_delta_ms(timer_u32() - timeAmphoursMinVoltage) >= 5000 && battery.current <= 5.0) {
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
            if (PotThrottleLevelReal == 0 && (battery.current < -0.05 && battery.current >= -0.5)) {
                battery.ampHoursUsed = 0;
                battery.wattHoursUsed = 0;
                if (battery.ampHoursRated_tmp != 0.0) {
                    battery.ampHoursRated = battery.ampHoursRated_tmp;
                }

                estimatedRangeReset();
            }
        }

        // Gears
        static int rotorTimeoutMs = 10000;
        static int rotorEnergiseTimeMs = 0;

        if (PotThrottleLevelReal < 1) {
            if (rotorCutOff_temp == true) {
                rotorCutOff_temp = false;
                timeRotorSleep = timer_u32();
            }

            if (timer_delta_ms(timer_u32() - timeRotorSleep) >= rotorTimeoutMs) {
                // if the erpm is lower than 50 -> send little power to rotor (maybe 1 watt?)
                // else send enough power to the rotor to track speed accurately
                if (VESC.data.rpm < 50.0f) {
                    // // this logic is so that the little current for tracking doesnt run all the time
                    // // it will power the rotor for 0.5s, then the next 4.5s there will be no power goind to the rotor, rinse and repeat
                    // static uint32_t timeRotorLittleCurrent = 0;
                    // if (timer_delta_ms(timer_u32() - timeRotorLittleCurrent) <= 500) {
                    //     setDuty(LEDC_CHANNEL_0, GearLevelIdleLittleCurrentDutyCycle);
                    // } else if (timer_delta_ms(timer_u32() - timeRotorLittleCurrent) > 500) {
                    //     setDuty(LEDC_CHANNEL_0, 0);
                    //     if (timer_delta_ms(timer_u32() - timeRotorLittleCurrent) >= 5000) {
                    //         timeRotorLittleCurrent = timer_u32();
                    //     }
                    // }
                    setDuty(LEDC_CHANNEL_0, GearLevelIdleLittleCurrentDutyCycle);
                    rotorCanPowerMotor = 0;
                } else {
                    setDuty(LEDC_CHANNEL_0, GearLevelTrackSpeedDutyCycle);
                }
            } else {
                if (settings.automaticGearChanging == 1) {
                    setGearLevel(3);
                }
            }
        } else {
            if (rotorCutOff_temp == false) {
                rotorCutOff_temp = true;
                timeRotorSleep = timer_u32();
            }

            if (timer_delta_ms(timer_u32() - timeRotorSleep) >= rotorEnergiseTimeMs) {
                rotorCanPowerMotor = 1;
            }

            if (settings.automaticGearChanging == 1) {
                if (VESC.data.avgMotorCurrent >= 110.0) { // PotThrottleAmpsRequested
                    setGearLevel(1);
                } else if (VESC.data.avgMotorCurrent >= 45.0) { // PotThrottleAmpsRequested
                    setGearLevel(2);
                } else {
                    setGearLevel(3);
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
        if (rotorCanPowerMotor == 0 || !POWER_ON) {
            VESC.setCurrent(0.0f);
        } else {
            VESC.setCurrent(throttle.map(PotThrottleLevelReal));
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
        wh_over_km_average = trip.wattHoursUsed / trip.distance;

        estimatedRange.AhPerKm = estimatedRange.ampHoursUsed / estimatedRange.distance;
        estimatedRange.range = (battery.ampHoursRated - battery.ampHoursUsed) / estimatedRange.AhPerKm;

        // Execute every second that elapsed
        if (timer_delta_ms(timer_u32() - timeExecEverySecondCore1) >= 1000) {
            timeExecEverySecondCore1 = timer_u32();
            // stuff to run

        }

        // Execute every 100ms that elapsed
        if (timer_delta_ms(timer_u32() - timeExecEvery100millisecondsCore1) >= 10000) {
            timeExecEvery100millisecondsCore1 = timer_u32();

            // VESC.setMcconfTempValues();
            // run_once_mcconf2 = true;

            // if (VESC.getMcconfTempValues()) {
            //     VESC.data_mcconf.l_current_max_scale = 0.3;
            //     VESC.data_mcconf.l_current_min_scale = 1.0;
            //     VESC.data_mcconf.l_max_erpm = 3.0; // 3k
            //     VESC.data_mcconf.l_min_erpm = -3.0; // -3k
            //     VESC.data_mcconf.l_min_duty = 0.005;
            //     VESC.data_mcconf.l_max_duty = 0.01;
            //     VESC.data_mcconf.l_watt_min = -1500.0;
            //     VESC.data_mcconf.l_watt_max = 200.0;

            //     VESC.setMcconfTempValues();
            //     delay(1000);
            // }
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
    wheel.gear_ratio = 14.9625f; // (42/10) * (57/16)
    battery.voltage_min = 66.0f;
    battery.voltage_max = 82.0f;

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
        {75, 120},
        {100, 200}
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

    // tftVSPI.begin(18, 19, 23, TFT_RST);

    tft.init(); // Init ST7789 320x240
    tft.setRotation(1); // 270 degrees rotation
    // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
    // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
    // may end up with a black screen some times, or all the time.
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setCursor(0, 0);

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

    // configure backlight of the TFT
    pinMode(     pinTFTbacklight, OUTPUT);
    digitalWrite(pinTFTbacklight, LOW); // Turn off backlight

    // Power Switch
    pinMode(pinPowerSwitch, INPUT_PULLDOWN);
    attachInterrupt(pinPowerSwitch, powerSwitchCallback, GPIO_INTR_ANYEDGE);
    powerSwitchCallback();

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = static_cast<ledc_timer_bit_t>(8),
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000, //1000
        .clk_cfg = LEDC_USE_APB_CLK,
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
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);

    for (int i = 0; i < 4; i++){
        buttonPressCount[i] = 2;
        timeButton[i] = 0;
    }

    powerLimiterPID.setOutputLimits(-100, 0);
    // powerLimiterPID.setSetpoint(50); // max power watts
    setPowerLevel(-1);
    setGearLevel(2);

    // setup ebike namespace
    preferences.begin("ebike", false);

    // retrieve values      
    odometer.distance       = preferences.getFloat("odometer", -1);
    battery.ampHoursUsed    = preferences.getFloat("batAhUsed", -1);
    battery.ampHoursRated   = preferences.getFloat("batAhFulDis", -1);
    battery.ampHoursUsedLifetime = preferences.getFloat("batAhUsedLife", 0);

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
                        commAddValue(&toSend, battery.amphours_min_voltage, 1);
                        commAddValue(&toSend, battery.amphours_max_voltage, 1);

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
                        commAddValue(&toSend, selectedPowerMode, 0);
                        commAddValue(&toSend, VESC.data.avgMotorCurrent, 1);
                        commAddValue(&toSend, wh_over_km_average, 1);
                        commAddValue(&toSend, estimatedRange.AhPerKm, 1);
                        commAddValue(&toSend, estimatedRange.range, 1);
                        commAddValue(&toSend, VESC.data.tempMotor, 1);
                        commAddValue(&toSend, totalSecondsSinceBoot, 0);
                        commAddValue(&toSend, timer_delta_us(timeCore0), 1);
                        commAddValue(&toSend, timer_delta_us(timeCore1), 1);
                        commAddValue(&toSend, acceleration, 1);
                        commAddValue(&toSend, POWER_ON, 0);
                   
                        toSend.append("\n");
                        break;

                    case COMMAND_ID::SET_ODOMETER:
                        odometer.distance = (float)getValueFromPacket(packet, 1);
                        break;

                    case COMMAND_ID::SAVE_PREFERENCES:
                        preferencesSaveBattery();
                        preferencesSaveOdometer();
                        break;
                        
                    case COMMAND_ID::RESET_TRIP:
                        tripReset();
                        break;

                    case COMMAND_ID::RESET_ESTIMATED_RANGE:
                        estimatedRangeReset();
                        break;

                    case COMMAND_ID::READY_TO_WRITE:
                        commAddValue(&toSend, COMMAND_ID::READY_TO_WRITE, 0);

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::GET_FW:
                        commAddValue(&toSend, COMMAND_ID::GET_FW, 0);
                        toSend.append(std::format("{};{}; {} {};", "EBIKE", "0.0", __DATE__, __TIME__)); // NAME, VERSION, COMPILE DATE/TIME

                        toSend.append("\n");
                        break;

                    case COMMAND_ID::PING:
                        display.ping();
                        break;

                    case COMMAND_ID::TOGGLE_FRONT_LIGHT:

                        break;

                    case COMMAND_ID::SET_AMPHOURS_USED_LIFETIME:
                        battery.ampHoursUsedLifetime = (float)getValueFromPacket(packet, 1);
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
                        VESC.data_mcconf.l_min_erpm = (float)getValueFromPacket(packet, 3) / 1360.82;
                        VESC.data_mcconf.l_max_erpm = (float)getValueFromPacket(packet, 4) / 1360.82;
                        VESC.data_mcconf.l_min_duty = (float)getValueFromPacket(packet, 5);
                        VESC.data_mcconf.l_max_duty = (float)getValueFromPacket(packet, 6);
                        VESC.data_mcconf.l_watt_min = (float)getValueFromPacket(packet, 7);
                        VESC.data_mcconf.l_watt_max = (float)getValueFromPacket(packet, 8);
                        VESC.data_mcconf.l_in_current_min = (float)getValueFromPacket(packet, 9);
                        VESC.data_mcconf.l_in_current_max = (float)getValueFromPacket(packet, 10);
                        VESC.data_mcconf.name = getValueFromPacket_string(packet, 11);
                        VESC.setMcconfTempValues();
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

        if (!display.isExternalDisplayConnected() && POWER_ON) {
            digitalWrite(pinTFTbacklight, HIGH); // Turn on backlight
            printDisplay();
        } else {
            digitalWrite(pinTFTbacklight, LOW); // Turn off backlight
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

            preferencesSaveBattery();

            if (odometer.distance_tmp != odometer.distance) {
                odometer.distance_tmp = odometer.distance;

                preferencesSaveOdometer();
            }
            Serial.printf("Preferences Saved\n");
        }
        
        timeCore0 = (timer_u32() - timeStartCore0);
    }
}
