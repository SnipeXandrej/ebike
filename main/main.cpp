#include <stdio.h>
#include <iostream>
#include <chrono>
#include <mutex>

// Arduino Libraries
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "HardwareSerial.h"
#include "Preferences.h"
#include <Adafruit_ADS1X15.h>
#include <VescUart.h>

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

#define TFT_CS         5
#define TFT_RST        -1
#define TFT_DC         32
// SPIClass tftVSPI = SPIClass(VSPI);
// Adafruit_ST7789 tft = Adafruit_ST7789(&tftVSPI, TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ADS1115 dedicatedADC;
VescUart VESC;

// Values From VESC
struct {
    float erpm;
    float tempMosfet;
    float avgMotorCurrent;
    float avgInputCurrent;
    float ampHours;
    float inputVoltage;
    float tachometer;
    float wattHours;
} VESCdata;

struct {
    float diameter;
    float gear_ratio;
} wheel;

struct {
    double tachometer_abs_now;
    double tachometer_abs_diff;
    double distance; // in km
    double DNU_refresh;
} trip;

struct {
    double distance_on_boot; // in km
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
    float watts;
    float wattHoursUsed;
    float percentage;
    float ampHoursFullyDischarged;
    float amphours_min_voltage;
    float amphours_max_voltage;
} battery;

float  _batteryVoltage;

float _batteryWattHoursUsedUsedInElapsedTime;

float  _batteryCurrent;
float _batteryCurrentUsedInElapsedTime;

float estimatedRangeLeft = 0;
float batteryampHoursUsed_tmp;

float  _potThrottleVoltage;
float potThrottleVoltage;

float wh_over_km = 0; // immediate
float wh_over_km_average = 0; // over time
float speed_kmh = 0;

uint32_t timeStartCore1 = 0, timeCore1 = 0;    
uint32_t timeStartCore0 = 0, timeCore0 = 0;    
uint32_t timeStartDisplay = 0; //timeDisplay = 0;
uint32_t timeExecEverySecondCore0 = 0;
uint32_t timeExecEverySecondCore1 = 0;
uint32_t timeExecEvery100millisecondsCore1 = 0;
uint32_t timeRotorSleep = 0;
unsigned long timeSavePreferencesStart = millis();

// settings
bool drawDebug = 0;
bool drawUptime = 1;
bool printCoreExecutionTime = 0;
bool disableOptimizedDrawing = 0;
bool batteryPercentageVoltageBased = 0;
bool cutMotorPower = 0;
bool cutRotorPower = 1;

bool rotorCanPowerMotor = 1;
bool rotorCutOff_temp = 1;

// uptime
float totalSecondsSinceBoot;
int clockSecondsSinceBoot, DNU_clockSecondsSinceBoot;
int clockMinutesSinceBoot;
int clockHoursSinceBoot;
int clockDaysSinceBoot;

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
float PotThrottleLevel;
float PotThrottleLevelPowerLimited;

char text[128];

std::string readString;

InputOffset BatVoltageCorrection;
InputOffset PotThrottleCorrection;

MovingAverage BatVoltageMovingAverage;
MovingAverage PotThrottleMovingAverage;
MovingAverage BatWattMovingAverage;
MovingAverage Throttle;
MovingAverage WhOverKmMovingAverage;
MovingAverage batteryAuxWattsMovingAverage;
MovingAverage batteryAuxMovingAverage;
MovingAverage batteryWattsMovingAverage;

Preferences preferences;

double kP = 0.2, kI = 0.1, kD = 2; //kP = 0.1, 0.3 is unstable
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

#define ADC_ATTEN      ADC_ATTEN_DB_12  // Allows reading up to 3.3V
#define ADC_WIDTH      ADC_WIDTH_BIT_12 // 12-bit resolution

// Enabling C++ compile
extern "C" { void app_main(); }

void preferencesSaveOdometer() {
    preferences.putFloat("odometer", (float)odometer.distance);
}

void preferencesSaveBattery() {
    preferences.putFloat("batAhUsed", battery.ampHoursUsed);
    preferences.putFloat("batAhFulDis", battery.ampHoursFullyDischarged);
}

float maxCurrentAtERPM(int erpm) {
    float ERPM_current_10 = 430.0f;
    float ERPM_current_100 = 1000.0f;
    float ERPM_current_180 = 3500.0f;

    if (erpm <= ERPM_current_10) {
        return 10.0f;
    } else if (erpm <= ERPM_current_100) {
        // Linear interpolation between 10A at 430 ERPM and 100A at 1000 ERPM
        float slope = (100.0f - 10.0f) / (ERPM_current_100 - ERPM_current_10);
        return slope * (erpm - ERPM_current_10);
    } else if (erpm < ERPM_current_180) {
        // Linear interpolation between 100A at 1000 ERPM and 180A at 3500 ERPM
        float slope = (180.0f - 100.0f) / (ERPM_current_180 - ERPM_current_100);
        return 100.0f + slope * (erpm - ERPM_current_100);
    } else {
        return 180.0f;
    }
}

void printVescMcConfTempValues() {
    Serial.printf("l_current_min_scale: %f\n", VESC.data_mcconf.l_current_min_scale);
    Serial.printf("l_current_max_scale: %f\n", VESC.data_mcconf.l_current_max_scale);
    Serial.printf("l_min_erpm: %f\n", VESC.data_mcconf.l_min_erpm);
    Serial.printf("l_max_erpm: %f\n", VESC.data_mcconf.l_max_erpm);
    Serial.printf("l_min_duty: %f\n", VESC.data_mcconf.l_min_duty);
    Serial.printf("l_max_duty: %f\n", VESC.data_mcconf.l_max_duty);
    Serial.printf("l_watt_min: %f\n", VESC.data_mcconf.l_watt_min);
    Serial.printf("l_watt_max: %f\n", VESC.data_mcconf.l_watt_max);
    Serial.printf("l_in_current_min: %f\n", VESC.data_mcconf.l_in_current_min);
    Serial.printf("l_in_current_min: %f\n", VESC.data_mcconf.l_in_current_min);
}

volatile bool dedicatedADC_new_data = false;
void IRAM_ATTR ADCNewDataReadyISR() {
  dedicatedADC_new_data = true;
}

uint32_t timerDedicatedADC = timer_u32();
void dedicatedADCDiff() {
    // If we don't have new data, skip this iteration.
    if (!dedicatedADC_new_data) {
        // while(!dedicatedADC_new_data);
        return;
    }

    int16_t results = dedicatedADC.getLastConversionResults();

    // AUX CURRENT
    battery.current = (((float)results) / 143.636); //143.636

    _batteryCurrentUsedInElapsedTime = battery.current / (1.0f / timer_delta_s(timer_u32() - timerDedicatedADC));
    battery.ampHoursUsed += _batteryCurrentUsedInElapsedTime / 3600.0;

    _batteryWattHoursUsedUsedInElapsedTime = (battery.current * battery.voltage) / (1.0f / timer_delta_s(timer_u32() - timerDedicatedADC));
    battery.wattHoursUsed += _batteryWattHoursUsedUsedInElapsedTime / 3600.0;

    dedicatedADC_new_data = false;
    // Serial.printf("Time it took to diff: %f\n", timer_delta_ms(timer_u32() - timerDedicatedADC));

    timerDedicatedADC = timer_u32();
}

int PowerLevelnegative1Watts = 1000000;
int PowerLevel0Watts = 100;
int PowerLevel1Watts = 250;
int PowerLevel2Watts = 500;
int PowerLevel3Watts = 1000;

void setPowerLevel(int level) {
    if (level == -1) {
        powerLimiterPID.setSetpoint(PowerLevelnegative1Watts);
        selectedPowerMode = -1;
    }

    if (level == 0) {
        powerLimiterPID.setSetpoint(PowerLevel0Watts);
        selectedPowerMode = 0;
    }

    if (level == 1) {
        powerLimiterPID.setSetpoint(PowerLevel1Watts);
        selectedPowerMode = 1;
    }

    if (level == 2) {
        powerLimiterPID.setSetpoint(PowerLevel2Watts);
        selectedPowerMode = 2;
    }

    if (level == 3) {
        powerLimiterPID.setSetpoint(PowerLevel3Watts);
        selectedPowerMode = 3;
    }
}

// rotor electromagnet DC resistance is around 3,05 ohms
int GearLevel0DutyCycle = 0; // 0W
int GearLevel1DutyCycle = 255; // 15V = 73.7W
int GearLevel2DutyCycle = 187; // 10.88V = 38.8W
int GearLevel3DutyCycle = 100; // 6.86V = 15.4W
int GearLevelIdleDutyCycle = GearLevel3DutyCycle; // There is some power so the VESC can track the speed
int GearLevelIdleLittleCurrentDutyCycle = 10; // There is some power so the VESC can track the speed
int GearDutyCycle;

void setGearLevel(int level) {
    if (level == 0) {
        selectedGear = 0;
        GearDutyCycle = GearLevel0DutyCycle;
    }

    if (level == 1) {
        selectedGear = 1;
        GearDutyCycle = GearLevel1DutyCycle;
    }

    if (level == 2) {
        selectedGear = 2;
        GearDutyCycle = GearLevel2DutyCycle;
    }

    if (level == 3) {
        selectedGear = 3;
        GearDutyCycle = GearLevel3DutyCycle;
    }

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GearDutyCycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
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
    clockSeconds += ((float)timer_delta_us(timeCore0) / 1000000);
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

void redrawScreen() {
    tft.fillScreen(ST77XX_BLACK);
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
    // tft.setCursor(268, 3); tft.printf("%3.0f%%", batteryPercentage);
    // Battery with amphours used
    tft.setCursor(150, 3); tft.printf("%5.3fAh %4.1f%%", battery.ampHoursUsed, battery.percentage); //batteryAmpHours

    // Upper Divider
    if (firsttime_draw) {
        tft.drawLine(0, 20, 320, 20, ST77XX_WHITE);
    }

    // Power Draw // Battery Voltage // Battery Amps draw
    tft.setCursor(3, 28); tft.printf("W:%6.1f  ", batteryWattsMovingAverage.moveAverage(battery.watts));
    tft.setCursor(3, 47); tft.printf("V:  %4.1f ", battery.voltage);
    tft.setCursor(3, 66); tft.printf("A:  %6.3f ", batteryAuxMovingAverage.moveAverage(battery.current));
    tft.setCursor(3, 85); tft.printf("vA: %6.3f vAp: %5.1f  ", VESCdata.avgInputCurrent, VESCdata.avgMotorCurrent); //vescCurrent, vescAmpHours
    tft.setCursor(3, 104); tft.printf("vAh: %5.3f", VESCdata.ampHours);
    // tft.setCursor(3, 123); tft.printf("Pot:%3.0f%%", PotThrottleLevel);
    // tft.setCursor(3, 142); tft.printf("CutRo: %d ", cutRotorPower);

    // tft.setCursor(3, 104); tft.printf("A: %6.3f  Ah: %5.4f  ", auxCurrent, auxAmpHours);
    // tft.setCursor(3, 85); tft.printf("A: %6.3f  ", vescCurrent);
    // tft.setCursor(3, 142); tft.printf("Pl: %3.0f%% ", PotThrottleLevelPowerLimited);
    // tft.setCursor(3, 124); tft.printf("Pot:%5.2f", potThrottle_voltage);

    // consumption over last 1km
    tft.setCursor(207-20, 28); tft.printf("%5.1f Wh/km", WhOverKmMovingAverage.moveAverage(wh_over_km));
    tft.setCursor(207-20, 28+19); tft.printf("%5.1f Wh/km", wh_over_km_average);
    tft.setCursor(207-20-40, 28+19+19); tft.printf("Range: %4.1f", estimatedRangeLeft);

    // Speed
    tft.setTextSize(5);
    tft.setCursor(125, 106); tft.printf("%4.1f", speed_kmh);
    tft.setCursor(248, 127); tft.setTextSize(2);
    if (firsttime_draw) {
        tft.println("km/h");
    }

    // Gear // Power Level
    if (firsttime_draw || (DNU_selectedGear != selectedGear) || (DNU_selectedPowerMode != selectedPowerMode)) {
        DNU_selectedGear = selectedGear;
        DNU_selectedPowerMode = selectedPowerMode;

        // tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
        tft.setCursor(125, 147); tft.printf("Gear:  %d", selectedGear);
        tft.setCursor(125, 166); tft.printf("Power: %d", selectedPowerMode);
        // tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    }

    // Uptime
    tft.setTextSize(1);
    if (drawUptime && (firsttime_draw || DNU_clockSecondsSinceBoot != clockSecondsSinceBoot))  {
        DNU_clockSecondsSinceBoot = clockSecondsSinceBoot;
        tft.setCursor(3, 205);
        tft.printf("Uptime: %2dd %2dh %2dm %2ds\n", clockDaysSinceBoot, clockHoursSinceBoot, clockMinutesSinceBoot, clockSecondsSinceBoot);
    }

    if (firsttime_draw) {
        // Bottom Divider
        tft.drawLine(0, 215, 320, 215, ST77XX_WHITE);
    }

    // Odometer
    if (firsttime_draw || odometer.DNU_refresh != odometer.distance) {
        odometer.DNU_refresh = odometer.distance;
        tft.setTextSize(2);
        tft.setCursor(3, 220);
        tft.printf("O: %.2f", odometer.distance);
    }

    // Trip
    if (firsttime_draw || trip.DNU_refresh != trip.distance) {
        trip.DNU_refresh = trip.distance;
        tft.setTextSize(2);
        tft.setCursor(220, 220);
        tft.printf("T: %.2f", trip.distance);
    }

    if (drawDebug) {
        tft.setTextSize(1);
        sprintf(text, "\nExecution time:"
                        "\n core0: %.1f us   "
                        "\n core1: %.1f us   \n",
                        timer_delta_us(timeCore0), timer_delta_us(timeCore1));
        tft.setCursor(160, 150);
        tft.println(text);
    }

    if (firsttime_draw) {
        firsttime_draw = disableOptimizedDrawing; // should be 0
    }
}

// runs on core 1
void loop_core1 (void* pvParameters) {
    while (1) {
        timeStartCore1 = timer_u32();

        // Reset temporary values
        _batteryVoltage = 0;
        _potThrottleVoltage = 0;

        // measure raw battery voltage from ADC
        for (int i = 0; i < 15; i++) {
            _batteryVoltage += adc1_get_raw(pinBatteryVoltage); // PIN 36/VP
            _potThrottleVoltage += adc1_get_raw(pinPotThrottle);

        }
        _batteryVoltage = _batteryVoltage / 15;
        _potThrottleVoltage = _potThrottleVoltage / 15;

        // BATTERY VOLTAGE
        battery.voltage = (36.11344125582536f * BatVoltageCorrection.correctInput(_batteryVoltage * (3.3f/4095.0f)));

        // BATTERY CURRENT
        dedicatedADCDiff();

        battery.watts = battery.voltage * battery.current;

        potThrottleVoltage = (
            PotThrottleCorrection.correctInput(
                PotThrottleMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _potThrottleVoltage
                )
            )
        );
        PotThrottleLevelReal = map_f(potThrottleVoltage, 0.2, 3.05, 0, 100);

        if ((int)PotThrottleLevelReal == 0) {
            if (rotorCutOff_temp == true) {
                rotorCutOff_temp = false;
                timeRotorSleep = timer_u32();
            }

            if (timer_delta_ms(timer_u32() - timeRotorSleep) >= 500) {
                rotorCanPowerMotor = 0;
                cutRotorPower = 1;

                if (GearDutyCycle == GearLevel0DutyCycle) {
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GearLevel0DutyCycle);
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
                } else if (VESCdata.erpm == 0) {
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GearLevelIdleLittleCurrentDutyCycle);
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
                } else {
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GearLevelIdleDutyCycle);
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
                }
            }
        } else {
            cutRotorPower = 0;
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, GearDutyCycle);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

            if (rotorCutOff_temp == false) {
                rotorCutOff_temp = true;
                timeRotorSleep = timer_u32();
            }

            if (timer_delta_ms(timer_u32() - timeRotorSleep) >= 0) { // can be 0 when there's always power going to the rotor
                rotorCanPowerMotor = 1;
            }
        }

        // for now, directly map voltage from the throttle to the pot input pin of VESC
        if (cutMotorPower || selectedGear == 0 || rotorCanPowerMotor == 0) {
            PotThrottleLevel = 0;
            PotThrottleLevelPowerLimited = 0;
            VESC.setCurrent(0.0f);
        } else {
            PotThrottleLevel = PotThrottleLevelReal;
            PotThrottleAdjustment = powerLimiterPID.getOutput(battery.watts);
            
            PotThrottleLevelPowerLimited = Throttle.moveAverage(PotThrottleLevel + PotThrottleAdjustment);
            VESC.setCurrent(map_f(PotThrottleLevel, 0, 100, 0, maxCurrentAtERPM(VESC.data.rpm))); //PotThrottleLevelPowerLimited
            // VESC.setCurrent(map_f(PotThrottleLevel, 0, 100, 0, 180)); //PotThrottleLevelPowerLimited
        }
        
        // Battery charge tracking stuff
        if (batteryPercentageVoltageBased) {
            battery.percentage = map_f(battery.voltage, battery.voltage_min, battery.voltage_max, 0, 100);
        } else {
            // TODO: implement amphour based battery percentage
            battery.percentage = map_f_nochecks(battery.ampHoursUsed, 0.0, battery.ampHoursFullyDischarged, 100.0, 0.0);
        }

        if (battery.voltage <= battery.amphours_min_voltage) {
            battery.ampHoursFullyDischarged = battery.ampHoursUsed;
        }

        if (battery.voltage >= battery.amphours_max_voltage) {
            battery.ampHoursUsed = 0;
            battery.wattHoursUsed = 0;
        }

        // Execute every second that elapsed
        if (timer_delta_ms(timer_u32() - timeExecEverySecondCore1) >= 1000) {
            timeExecEverySecondCore1 = timer_u32();

            // stuff to run
        }

        // Execute every 100ms that elapsed
        if (timer_delta_ms(timer_u32() - timeExecEvery100millisecondsCore1) >= 100) {
            timeExecEvery100millisecondsCore1 = timer_u32();

            // stuff to run
        }

        if (VESC.getVescValues()) {
            VESCdata.erpm = VESC.data.rpm;
            VESCdata.avgMotorCurrent = VESC.data.avgMotorCurrent;
            VESCdata.tachometer = VESC.data.tachometerAbs;
            VESCdata.avgInputCurrent = VESC.data.avgInputCurrent;
            VESCdata.inputVoltage = VESC.data.inpVoltage;
            VESCdata.wattHours = VESC.data.wattHours;
            VESCdata.ampHours = VESC.data.ampHours;

            if (trip.tachometer_abs_now < VESCdata.tachometer) {
                trip.tachometer_abs_diff = VESCdata.tachometer - trip.tachometer_abs_now;

                trip.distance += ((trip.tachometer_abs_diff / (double)motor.poles) / (double)wheel.gear_ratio) * (double)wheel.diameter * 3.14159265 / 100000.0; // divide by 100000 for trip distance to be in kilometers

                trip.tachometer_abs_now = VESCdata.tachometer;
            }

            speed_kmh = (((float)VESCdata.erpm / (float)motor.magnetPairs) / wheel.gear_ratio) * wheel.diameter * 3.14159265f * 60.0f/*minutes*/ / 100000.0f/*1 km in cm*/;
        }
        odometer.distance = odometer.distance_on_boot + trip.distance;

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
        wh_over_km_average = battery.wattHoursUsed / trip.distance;


        estimatedRangeLeft = (trip.distance / (battery.ampHoursUsed - batteryampHoursUsed_tmp)) * (battery.ampHoursFullyDischarged - batteryampHoursUsed_tmp);

        timeCore1 = (timer_u32() - timeStartCore1);
    }
}

// runs on core 0
void app_main(void)
{
    motor.poles = 36;
    motor.magnetPairs = 6;
    wheel.diameter = 63.0f;
    wheel.gear_ratio = 5.7f;
    battery.voltage_min = 62.0f;
    battery.voltage_max = 82.0f;

    battery.amphours_min_voltage = 64.0f;
    battery.amphours_max_voltage = 83.0f;

    initArduino();
    Serial.begin(115200);

    Serial2.begin(38400, SERIAL_8N1, 16, 17); // RX/TX
    VESC.setSerialPort(&Serial2);

    Wire.begin(21, 22);
    Wire.setClock(400000); // 400kHz

    if (!dedicatedADC.begin()) {
        Serial.println("Failed to initialize ADS.");
        // while (1);
    }
    // Up-to 64SPS is noise-free, 128SPS has a little noise, 250 and beyond is noise af
    dedicatedADC.setDataRate(RATE_ADS1115_64SPS);
    dedicatedADC.setGain(GAIN_SIXTEEN);
    pinMode(pinAdcRdyAlert, INPUT);
    // We get a falling edge every time a new sample is ready.
    attachInterrupt(pinAdcRdyAlert, ADCNewDataReadyISR, FALLING);
    dedicatedADC.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);

    // tftVSPI.begin(18, 19, 23, TFT_RST);

    tft.init(240, 320); // Init ST7789 320x240
    tft.setRotation(3); // 270 degrees rotation
    // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
    // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
    // may end up with a black screen some times, or all the time.
    tft.setSPISpeed(40000000);
    tft.invertDisplay(false);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(0, 0);

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
    Throttle.smoothingFactor = 0.1;
    WhOverKmMovingAverage.smoothingFactor = 0.25;
    batteryAuxWattsMovingAverage.smoothingFactor = 0.2;
    batteryAuxMovingAverage.smoothingFactor = 0.2;
    batteryWattsMovingAverage.smoothingFactor = 0.2;

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
    pinMode(     pinTFTbacklight, OUTPUT); // Backlight of TFT
    digitalWrite(pinTFTbacklight, HIGH); // Turn on backlight

    setCpuFrequencyMhz(240);

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
        .duty = 127,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);

    for (int i = 0; i < 4; i++){
        buttonPressCount[i] = 2;
        timeButton[i] = 0;
    }

    // powerLimiterPID.setOutputLimits(-100, 0);
    // powerLimiterPID.setSetpoint(50); // max power watts
    setPowerLevel(-1);
    setGearLevel(1);

    // setup ebike namespace
    preferences.begin("ebike", false);

    // // store values
    // preferencesSave();

    // retrieve values      
    odometer.distance_on_boot       = preferences.getFloat("odometer", -1);
    battery.ampHoursUsed            = preferences.getFloat("batAhUsed", -1);
    battery.ampHoursFullyDischarged = preferences.getFloat("batAhFulDis", -1);

    batteryampHoursUsed_tmp = battery.ampHoursUsed;

    odometer.distance_tmp = odometer.distance_on_boot;

    delay(2000);

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

        while (Serial.available()) {
          // delayMicroseconds(250); // B115200
          // delayMicroseconds(30000); // B4800
          delayMicroseconds(250);
          char c = Serial.read();
          readString += c;
        }

        if (!readString.empty()) {
            // TODO: make these "contains" into some kind of function, so the code doesn't need to be repeated

            // Serial.printf("text %s\n", readString.c_str());

            if (readString.contains("displayRefresh\n"))
                redrawScreen();

            if (readString.contains("clockHours="))
                clockHours = getValueFromString("clockHours", readString);

            if (readString.contains("clockMinutes="))
                clockMinutes = getValueFromString("clockMinutes", readString);

            if (readString.contains("printCoreExecutionTime="))
                printCoreExecutionTime = getValueFromString("printCoreExecutionTime", readString);

            if (readString.contains("disableOptimizedDrawing="))
            {
                disableOptimizedDrawing = getValueFromString("disableOptimizedDrawing", readString);
                redrawScreen();
            }

            if (readString.contains("gear="))
                setGearLevel((int)getValueFromString("gear", readString));

            if (readString.contains("kP="))
            {
                kP = (double)getValueFromString("kP", readString);
                // Serial.printf("kP: %0.3lf \n", kP);
                powerLimiterPID.setPID(kP, kI, kD);
            }

            if (readString.contains("kI="))
            {
                kI = (double)getValueFromString("kI", readString);
                // Serial.printf("kI: %0.3lf \n", kI);
                powerLimiterPID.setPID(kP, kI, kD);
            }

            if (readString.contains("kD="))
            {
                kD = (double)getValueFromString("kD", readString);
                // Serial.printf("kD: %0.3lf \n", kI);
                powerLimiterPID.setPID(kP, kI, kD);
            }

            if (readString.contains("powerMode="))
                setPowerLevel((int)getValueFromString("powerMode", readString));
            
            if (readString.contains("drawDebug="))
            {
                drawDebug = getValueFromString("drawDebug", readString);
                redrawScreen();
            }

            if (readString.contains("drawUptime="))
            {
                drawUptime = getValueFromString("drawUptime", readString);
                redrawScreen();
            }

            if (readString.contains("motorCurrent="))
            {
                float VESC_setCurrent = getValueFromString("motorCurrent", readString);
                VESC.setCurrent(VESC_setCurrent);
            }

            if (readString.contains("save"))
            {
                preferencesSaveOdometer();
                preferencesSaveBattery();
                Serial.printf("Preferences were saved\n");
            }

            if (readString.contains("odometer="))
            {
                odometer.distance_on_boot = (double)getValueFromString("odometer", readString);
            }

            if (readString.contains("trip="))
            {
                trip.distance = (double)getValueFromString("trip", readString);
            }

            if (readString.contains("ampHoursFullyDischarged="))
            {
                battery.ampHoursFullyDischarged = getValueFromString("ampHoursFullyDischarged", readString);
                Serial.printf("battery.ampHoursFullyDischarged=%f\n", battery.ampHoursFullyDischarged);
            }

            if (readString.contains("getBatteryStats"))
            {
                Serial.printf("battery.amphours_min_voltage=%f\n", battery.amphours_min_voltage);
                Serial.printf("battery.amphours_max_voltage=%f\n", battery.amphours_max_voltage);
                Serial.printf("battery.ampHoursFullyDischarged=%f\n", battery.ampHoursFullyDischarged);
                Serial.printf("battery.ampHoursUsed=%f\n", battery.ampHoursUsed);
                Serial.printf("battery.current=%f\n", battery.current);
                Serial.printf("battery.percentage=%f\n", battery.percentage);
                Serial.printf("battery.voltage=%f\n", battery.voltage);
                Serial.printf("battery.voltage_min=%f\n", battery.voltage_min);
                Serial.printf("battery.voltage_max=%f\n", battery.voltage_max);
                Serial.printf("battery.wattHoursUsed=%f\n", battery.wattHoursUsed);
                Serial.printf("battery.watts=%f\n", battery.watts);
            }

            if (readString.contains("printVescMcconf"))
            {
                if (!VESC.getMcconfTempValues()) {
                    Serial.printf("Failed to get Temp McConf values\n");
                } else {
                    printVescMcConfTempValues();
                }
            }

            if (readString.contains("power1"))
            {
                VESC.data_mcconf.l_current_min_scale = 1.0;
                VESC.data_mcconf.l_current_max_scale = 1.0;
                VESC.data_mcconf.l_min_erpm = -100000.0;
                VESC.data_mcconf.l_max_erpm = 27000;
                VESC.data_mcconf.l_min_duty = 0.005;
                VESC.data_mcconf.l_max_duty = 0.95;
                VESC.data_mcconf.l_watt_min = -1000.0;
                VESC.data_mcconf.l_watt_max = 250.0;
                VESC.data_mcconf.l_in_current_min = -12.0;
                VESC.data_mcconf.l_in_current_max = -12.0;

                VESC.setMcconfTempValues();
            }

            readString="";
        }

        totalSecondsSinceBoot += ((float)timer_delta_us(timeCore0) / 1000000);
        clockSecondsSinceBoot = (int)(totalSecondsSinceBoot) % 60;
        clockMinutesSinceBoot = (int)(totalSecondsSinceBoot / 60) % 60;
        clockHoursSinceBoot   = (int)(totalSecondsSinceBoot / 60 / 60) % 24;
        clockDaysSinceBoot    = totalSecondsSinceBoot / 60 / 60 / 24;

        // function for counting the current time and date
        clock_date_and_time();

        printDisplay();

        // Execute every second that elapsed
        if (timer_delta_ms(timer_u32() - timeExecEverySecondCore0) >= 1000) {
            timeExecEverySecondCore0 = timer_u32();

            // stuff to run
            if (printCoreExecutionTime) {
                Serial.printf("\n\rExecution time:"
                            "\n\r core0: %.1f us   "
                            "\n\r core1: %.1f us   "
                            "\n\r timer_u32(): %llu ns                      \n\r",
                            timer_delta_us(timeCore0), timer_delta_us(timeCore1), timer_u32());
            }
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
