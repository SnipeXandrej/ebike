#include <stdio.h>
#include <iostream>
#include <chrono>

// Arduino Libraries
#include "Arduino.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "HardwareSerial.h"
#include "Preferences.h"

#include "esp32-hal-dac.h"
#include "esp32-hal-ledc.h"
#include "hal/dac_types.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "inputOffset.h"
#include "fastGPIO.h"
#include "timer_u32.h"
#include "MiniPID.h"

bool firsttime_draw = 1;

#define TFT_CS         5
#define TFT_RST        -1
#define TFT_DC         32
// SPIClass tftVSPI = SPIClass(VSPI);
// Adafruit_ST7789 tft = Adafruit_ST7789(&tftVSPI, TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

float  _batteryVoltage;
float batteryVoltage;

float  _batteryCurrent;
float batteryCurrent;

float batteryWattConsumption;

float  _vescCurrent;
float vescCurrent;

float  _auxCurrent;
float auxCurrent;

float  _potThrottleVoltage;
float potThrottleVoltage;

float batteryPercentage;

float odometer, DNU_odometer_refresh; 
float trip, DNU_trip_refresh;

float speedkmh;
float wh_over_km = 0;

uint32_t timeStartCore1 = 0;    uint32_t timeStartCore0 = 0;    uint32_t timeStartDisplay = 0;
uint32_t timeEndCore1 = 0;      uint32_t timeEndCore0 = 0;      uint32_t timeEndDisplay = 0;
uint32_t timeCore1 = 0;         uint32_t timeCore0 = 0;         uint32_t timeDisplay = 0;

// uint32_t timeStartCutMotorPowerTimeout = 0 ,timeCutMotorPowerTimeout = 0;

int core0loopcount = 0;
int core1loopcount = 0;

// settings
bool drawDebug = 0;
bool drawUptime = 1;
bool printCoreExecutionTime = 0;
bool disableOptimizedDrawing = 0;
bool batteryPercentageVoltageBased = 1;
bool cutMotorPower = 0;

// settings clock
float totalSecondsSinceBoot;
int clockSecondsSinceBoot, DNU_clockSecondsSinceBoot;
int clockMinutesSinceBoot;
int clockHoursSinceBoot;
int clockDaysSinceBoot;

int clockHours = 21, DNU_clockHours;
int clockMinutes = 37, DNU_clockMinutes;

// settings for gearing and power
int selectedGear, DNU_selectedGear;
int selectedPowerMode, DNU_selectedPowerMode;


// char text[128];
char text2[128];

std::string readString;

InputOffset BatVoltageCorrection;
InputOffset AuxCurrentCorrection;
InputOffset PotThrottleCorrection;
InputOffset VESCCurrentCorrection;

MovingAverage BatVoltageMovingAverage;
MovingAverage AuxCurrentMovingAverage;
MovingAverage PotThrottleMovingAverage;
MovingAverage VESCCurrentMovingAverage;

MovingAverage Throttle;

Preferences preferences;

double PotThrottleAdjustment;
double PotThrottleLevel;
double PotThrottleLevelPowerLimited;

double kP = 0.2, kI = 0.1, kD = 2; //kP = 0.1, 0.3 is unstable
MiniPID powerLimiterPID(kP, kI, kD);

#define pinRotor          13                // D13
#define pinBatteryVoltage ADC1_CHANNEL_0    // D36
#define pinAuxCurrent     ADC1_CHANNEL_3    // D39
#define pinPotThrottle    ADC1_CHANNEL_6    // D34
#define pinVESCCurrent    ADC1_CHANNEL_7    // D35
#define pinOutToVESC      25                // D25
#define pinTFTbacklight   2                 // D2

#define ADC_ATTEN      ADC_ATTEN_DB_12  // Allows reading up to 3.3V
#define ADC_WIDTH      ADC_WIDTH_BIT_12 // 12-bit resolution

// Enabling C++ compile
extern "C" { void app_main(); }

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

int GearLevel1DutyCycle = 20;
int GearLevel2DutyCycle = 10;
int GearLevel3DutyCycle = 5;
int GearDutyCycle;

void setGearLevel(int level) {
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

    ledcWrite( pinRotor, GearDutyCycle);
}

float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    int temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (temp < out_min)
        temp = out_min;

    if (temp > out_max)
        temp = out_max;

    return temp;
}

std::string removeStringWithEqualSignAtTheEnd(const std::string toRemove, std::string str) {
    size_t pos = str.find(toRemove);
    str.erase(pos, toRemove.length() + 1);

    // cout << str << "\n";
    return str;
}

float getValueFromString(const std::string toRemove, std::string str) {
    float value;

    value = stof(removeStringWithEqualSignAtTheEnd(toRemove, str));

    // try {

    // }
    // catch (std::invalid_argument const& ex) {
    //     std::cout << "this did an oopsie: " << ex.what() << '\n';
    // }

    return value;
}

void redrawScreen() {
    tft.fillScreen(ST77XX_BLACK);
    firsttime_draw = 1;
}

// void printDebug() {
//         sprintf(text, "RAW:       %.04dV"
//               "\nCorrected: %.04fV"
//               "\n"
//               "\nExecution time:"
//               "\n core0: %lu us   "
//               "\n core1: %lu us   \n",
//               _batteryVoltage, batteryVoltage, timeCore0, timeCore1);

//         tft.setTextSize(2);
//         tft.setCursor(0, 0);
//         tft.println(text);
// }

void printDisplay() {
    // TODO: only update the variables, not the whole text, as it takes longer to draw

    // Clock
    tft.setTextSize(2);
    if (firsttime_draw || (DNU_clockMinutes != clockMinutes) || (DNU_clockHours != clockHours)) {
        DNU_clockMinutes = clockMinutes;
        DNU_clockHours = clockHours;
        tft.setCursor(3, 3); tft.printf("%02d:%02d", clockHours, clockMinutes);
    }

    // Battery
    tft.setCursor(268, 3); tft.printf("%3.0f%%", batteryPercentage);

    // Upper Divider
    if (firsttime_draw) {
        tft.drawLine(0, 20, 320, 20, ST77XX_WHITE);
    }

    // Power Draw // Battery Voltage // Battery Amps draw
    tft.setCursor(3, 28); tft.printf("W: %5.0f  ", batteryWattConsumption);
    tft.setCursor(3, 47); tft.printf("V: %5.2f  ", batteryVoltage);
    tft.setCursor(3, 66); tft.printf("A: %5.2f  ", auxCurrent);
    tft.setCursor(3, 85); tft.printf("A: %5.2f  ", vescCurrent);
    tft.setCursor(3, 104); tft.printf("Pot:%3.0f%%", PotThrottleLevel);
    tft.setCursor(3, 123); tft.printf("Pl: %3.0f%% ", PotThrottleLevelPowerLimited);
    // tft.setCursor(3, 124); tft.printf("Pot:%5.2f", potThrottle_voltage);

    // consumption over last 1km
    tft.setCursor(207, 28); tft.printf("%3.1f Wh/km", wh_over_km);

    // Speed
    tft.setTextSize(5);
    tft.setCursor(125, 106); tft.printf("%.0f", speedkmh);
    tft.setCursor(185, 127); tft.setTextSize(2);
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
    if (firsttime_draw || DNU_odometer_refresh != odometer) {
        DNU_odometer_refresh = odometer;
        tft.setTextSize(2);
        tft.setCursor(3, 220);
        tft.printf("O: %.1f", odometer);
    }

    // Trip
    if (firsttime_draw || DNU_trip_refresh != trip) {
        DNU_trip_refresh = trip;
        tft.setCursor(220, 220);
        tft.printf("T: %.1f", trip);
    }


    if (drawDebug) {
        tft.setTextSize(1);
        sprintf(text2, "\nExecution time:"
                        "\n core0: %.1f us   "
                        "\n core1: %.1f us   \n",
                        timer_delta_us(timeCore0), timer_delta_us(timeCore1));
        tft.setCursor(160, 150);
        tft.println(text2);
    }

    if (firsttime_draw) {
        firsttime_draw = disableOptimizedDrawing; // should be 0
    }
}

// runs on core 1
void loop_core1 (void* pvParameters) {
    while (1) {
        // if (core1loopcount == 0)
            timeStartCore1 = timer_u32();

        // Reset temporary values
        _batteryVoltage = 0;
        _auxCurrent = 0;
        _potThrottleVoltage = 0;
        _vescCurrent = 0;

        for (int i = 0; i < 5; i++) {
            _batteryVoltage += adc1_get_raw(pinBatteryVoltage); // PIN 36/VP
            _auxCurrent += adc1_get_raw(pinAuxCurrent); // PIN 39/VN
            _potThrottleVoltage += adc1_get_raw(pinPotThrottle);
            _vescCurrent += adc1_get_raw(pinVESCCurrent); // PIN 35
        }
        _batteryVoltage = _batteryVoltage / 5;
        _auxCurrent = _auxCurrent / 5;
        _potThrottleVoltage = _potThrottleVoltage / 5;
        _vescCurrent = _vescCurrent / 5;

        // BATTERY VOLTAGE
        batteryVoltage = (29.963f *
            BatVoltageCorrection.correctInput(
                BatVoltageMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _batteryVoltage
                )
            )
        ); //29.839

        // AUX CURRENT
        _auxCurrent = (
            AuxCurrentCorrection.correctInput(
                AuxCurrentMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _auxCurrent
                )
            )
        );
        _auxCurrent = ((((float)953+(float)560)/(float)953) * _auxCurrent);
        auxCurrent = (_auxCurrent - (float)2.545) / (float)0.192;

        // AUX CURRENT
        _vescCurrent = (
            VESCCurrentCorrection.correctInput(
                VESCCurrentMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _vescCurrent
                )
            )
        );
        _vescCurrent = ((((float)965+(float)560)/(float)965) * _vescCurrent);
        vescCurrent = (_vescCurrent - (float)2.529) / (float)0.116; //122
        // vescCurrent = _vescCurrent;

        // Battery Current
        batteryCurrent = auxCurrent + vescCurrent;

        // Battery Watt Consumption
        batteryWattConsumption = batteryVoltage * batteryCurrent;

        // Throttle Voltage
        potThrottleVoltage = (
            PotThrottleCorrection.correctInput(
                PotThrottleMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _potThrottleVoltage
                )
            )
        );

        // if (selectedPowerMode == 1) {
        //     if (batteryWattConsumption > 100) {
        //         cutMotorPower = 1;
        //         timeStartCutMotorPowerTimeout = timer_u32();
        //     } else {
        //         if (timer_delta_ms(timer_u32() - timeStartCutMotorPowerTimeout) > 50) {
        //             cutMotorPower = 0;
        //         } else {

        //         }
        //     }
        // }

        // for now, directly map voltage from the throttle to the pot input pin of VESC
        if (cutMotorPower) {
            dacWrite(pinOutToVESC, 0); //no power
        } else {
            PotThrottleLevel = map_f(potThrottleVoltage, 0, 3.3, 0, 100);
            PotThrottleAdjustment = powerLimiterPID.getOutput(batteryWattConsumption);
            
            PotThrottleLevelPowerLimited = Throttle.moveAverage(PotThrottleLevel + PotThrottleAdjustment);
            dacWrite(pinOutToVESC, (int)map_f(PotThrottleLevelPowerLimited, 0, 100, 0, 255));
        }
        

        if (batteryPercentageVoltageBased) {
            batteryPercentage = map_f(batteryVoltage, 34, 41, 0, 100);
        } else {
            // TODO: implement amphour based battery percentage
            batteryPercentage = 0;
        }

        if (core1loopcount < 200) {
            core1loopcount++;
        } else {
            core1loopcount = 0;
            // timeCore1 = (timer_u32() - timeStartCore1) / 200;
        }
        timeCore1 = (timer_u32() - timeStartCore1);
    }
}

// runs on core 0
void app_main(void)
{
    initArduino();
    Serial.begin(460800);

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
    // input, offset // 5/10/20 are the DAC values I used to use as an adjustable input to the ADC
    {0.0, 0},
    {0.0046, 0.1234}, // 5
    {0.0519, 0.1331}, // 10
    {0.1629, 0.1401}, // 20
    {0.2754, 0.1456}, // 30
    {0.3595, 0.149}, // 37
    {0.3918, 0.1512}, // 40
    {0.5095, 0.152}, // 50
    {0.5665, 0.1545}, // 55
    {0.623, 0.156}, // 60
    {0.677, 0.157}, // 65
    {0.7365, 0.1485}, // 70
    {0.7935, 0.1595}, // 75
    {0.852, 0.159}, // 80
    {0.9685, 0.1605}, // 90
    {1.089, 0.165}, // 100
    {1.204, 0.168}, // 110
    {1.3195, 0.1685}, // 120
    {1.465, 0.173}, // 130
    {1.58, 0.178}, // 140
    {1.6969, 0.1791}, // 150
    {1.81, 0.1845}, // 160
    {1.9257, 0.1903}, // 170
    {2.036, 0.199}, // 180
    {2.1585, 0.1945}, // 190
    {2.2758, 0.1917}, // 200
    {2.408, 0.178}, // 210
    {2.563, 0.142}, // 220
    {2.7408, 0.0872}, // 230
    {2.93, 0.014}, // 240
    {3.1352, -0.0772}, // 250
    {3.2415, -0.1295}, // 255
    {3.3, 0}
    };

    AuxCurrentCorrection.offsetPoints = BatVoltageCorrection.offsetPoints;
    PotThrottleCorrection.offsetPoints = BatVoltageCorrection.offsetPoints;
    VESCCurrentCorrection.offsetPoints = BatVoltageCorrection.offsetPoints;

    // BatVoltageCorrection.smoothingFactor = 1;
    BatVoltageMovingAverage.smoothingFactor = 0.2; //0.2

    // BatCurrentCorrection.smoothingFactor = 1;
    AuxCurrentMovingAverage.smoothingFactor = 0.2; // 0.2

    // PotThrottleCorrection.smoothingFactor = 1;
    PotThrottleMovingAverage.smoothingFactor = 0.5; // 0.5

    VESCCurrentMovingAverage.smoothingFactor = 0.2; // 0.5

    Throttle.smoothingFactor = 0.1;


    // Configure ADC width (resolution)
    adc1_config_width(ADC_WIDTH);
    // Configure ADC channel and attenuation
    adc1_config_channel_atten(pinBatteryVoltage, ADC_ATTEN); // PIN 36/VP
    adc1_config_channel_atten(pinAuxCurrent,     ADC_ATTEN); // PIN 39/VN
    adc1_config_channel_atten(pinPotThrottle,    ADC_ATTEN); // PIN 34
    adc1_config_channel_atten(pinVESCCurrent,    ADC_ATTEN); // PIN 35

    pinMode( pinOutToVESC, OUTPUT);
    dacWrite(pinOutToVESC, 0);

    pinMode(     pinTFTbacklight, OUTPUT); // Backlight of TFT
    digitalWrite(pinTFTbacklight, HIGH); // Turn on backlight

    ledcAttach(pinRotor, 5000, 8);
    ledcWrite( pinRotor, 0);


    powerLimiterPID.setOutputLimits(-100, 0);
    // powerLimiterPID.setSetpoint(50); // max power watts
    setPowerLevel(-1);
    setGearLevel(3);


    // setup ebike namespace
    preferences.begin("ebike", false);
    // retrieve values      
    odometer = preferences.getFloat("odometer", -1);
    trip     = preferences.getFloat("trip", -1);

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
        if (core0loopcount == 0) {
            // Save values
            // preferences.putFloat("odometer", odometer);
            // preferences.putFloat("trip", trip);
            // Serial.println("Preferences saved!\n");
        }
        timeStartCore0 = timer_u32();

        while (Serial.available()) {
          // delayMicroseconds(250); // B115200
          // delayMicroseconds(30000); // B4800
          delayMicroseconds(250);
          char c = Serial.read();
          readString += c;
        }

        if (!readString.empty()) {
            // Serial.printf("text %s\n", readString.c_str());

            if (readString.contains("displayRefresh\n"))
            {
                redrawScreen();
            }

            if (readString.contains("clockHours="))
            {
                clockHours = getValueFromString("clockHours", readString);
            }

            if (readString.contains("clockMinutes="))
            {
                clockMinutes = getValueFromString("clockMinutes", readString);
            }

            if (readString.contains("printCoreExecutionTime="))
            {
                printCoreExecutionTime = getValueFromString("printCoreExecutionTime", readString);
            }

            if (readString.contains("disableOptimizedDrawing="))
            {
                disableOptimizedDrawing = getValueFromString("disableOptimizedDrawing", readString);
                redrawScreen();
            }

            if (readString.contains("gear="))
            {
                setGearLevel((int)getValueFromString("gear", readString));
            }

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
            {
                setPowerLevel((int)getValueFromString("powerMode", readString));
            }

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

            // if (readString.contains("save"))
            // {
            //     clockMinutes = getValueFromString("clockMinutes", readString);
            // }

            readString="";
        }


        totalSecondsSinceBoot += ((float)timer_delta_us(timeCore0) / 1000000);
        clockSecondsSinceBoot = (int)(totalSecondsSinceBoot) % 60;
        clockMinutesSinceBoot = (int)(totalSecondsSinceBoot / 60) % 60;
        clockHoursSinceBoot =   (int)(totalSecondsSinceBoot / 60 / 60) % 24;
        clockDaysSinceBoot =    totalSecondsSinceBoot / 60 / 60 / 24;


        // if ((timeEndDisplay - timeStartDisplay) > 66) { // 15Hz
        printDisplay();


          // totalSecondsSinceBoot = esp_timer_get_time() / 1000000; // +353000 = 4d 2h 3m 20s


          // Serial.printf("Uptime: %02d:%02d:%02d:%02d\n", daysSinceBoot, hoursSinceBoot, minutesSinceBoot, secondsSinceBoot);


          // timeStartDisplay = millis();
        // }
        // timeEndDisplay = millis();

        if (core0loopcount < 20) {
            core0loopcount++;
        } else {
            core0loopcount = 0;

            if (printCoreExecutionTime) {
                Serial.printf("\n\rExecution time:"
                            "\n\r core0: %.1f us   "
                            "\n\r core1: %.1f us   "
                            "\n\r timer_u32(): %llu ns                      \n\r",
                            timer_delta_us(timeCore0), timer_delta_us(timeCore1), timer_u32());
            }
        }
            timeCore0 = (timer_u32() - timeStartCore0);

    }
}
