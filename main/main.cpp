#include <stdio.h>
#include "Arduino.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "HardwareSerial.h"

#include <iostream>
#include <chrono>

#include "esp32-hal-dac.h"
#include "esp32-hal-ledc.h"
#include "hal/dac_types.h"
#include "inputOffset.h"
#include "fastGPIO.h"
// #include "timer.h"
#include "timer_u32.h"

#include "Preferences.h"

Preferences preferences;

bool firsttime_draw = 1;

#define TFT_CS         5
#define TFT_RST        -1
#define TFT_DC         32
// SPIClass tftVSPI = SPIClass(VSPI);
// Adafruit_ST7789 tft = Adafruit_ST7789(&tftVSPI, TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int  _batteryVoltage;
float batteryVoltage;

int  _batteryCurrent;
float batteryCurrent;

int  _potThrottleVoltage;
float potThrottleVoltage;

float batteryPercentage;
bool  batteryPercentageVoltageBased = 1;

float odometer;
float trip;
float speedkmh;

bool printDebugStuff = 1;
bool printUptime = 1;

InputOffset BatVoltageCorrection;
InputOffset BatCurrentCorrection;
InputOffset PotThrottleCorrection;

MovingAverage BatVoltageMovingAverage;
MovingAverage BatCurrentMovingAverage;
MovingAverage PotThrottleMovingAverage;

uint32_t timeStartCore1 = 0;    uint32_t timeStartCore0 = 0;    uint32_t timeStartDisplay = 0;
uint32_t timeEndCore1 = 0;      uint32_t timeEndCore0 = 0;      uint32_t timeEndDisplay = 0;
uint32_t timeCore1 = 0;         uint32_t timeCore0 = 0;         uint32_t timeDisplay = 0;

int core0loopcount = 0;
int core1loopcount = 0;

float totalSecondsSinceBoot;
int secondsSinceBoot;
int minutesSinceBoot;
int hoursSinceBoot;
int daysSinceBoot;

int selected_gear = 1;
int selected_power_mode = 1;
float wh_over_km = 0;

int clockHours = 21;
int clockMinutes = 37;

char text[128];
char text2[128];

std::string readString;

#define pinRotor          13                // D13
#define pinBatteryVoltage ADC1_CHANNEL_0    // D36
#define pinBatteryCurrent ADC1_CHANNEL_3    // D39
#define pinPotThrottle    ADC1_CHANNEL_6    // D34
#define pinOutToVESC      25                // D25
#define pinTFTbacklight   2                 // D2

#include "driver/adc.h"
#include "esp_adc_cal.h"
#define ADC_ATTEN      ADC_ATTEN_DB_12  // Allows reading up to 3.3V
#define ADC_WIDTH      ADC_WIDTH_BIT_12 // 12-bit resolution

// Enabling C++ compile
extern "C" { void app_main(); }

float map(float x, float in_min, float in_max, float out_min, float out_max) {
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

    value = stoi(removeStringWithEqualSignAtTheEnd(toRemove, str));

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

void printDebug() {
        sprintf(text, "RAW:       %.04dV"
              "\nCorrected: %.04fV"
              "\n"
              "\nExecution time:"
              "\n core0: %lu us   "
              "\n core1: %lu us   \n",
              _batteryVoltage, batteryVoltage, timeCore0, timeCore1);

        tft.setTextSize(2);
        tft.setCursor(0, 0);
        tft.println(text);
}

void printDisplay() {
    // TODO: only update the variables, not the whole text, as it takes longer to draw

    // Clock
    tft.setTextSize(2);
    tft.setCursor(3, 3); tft.printf("%02d:%02d", clockHours, clockMinutes);

    // Battery
    tft.setCursor(268, 3); tft.printf("%3.0f%%", batteryPercentage);

    if (firsttime_draw) {
        // Upper Divider
        tft.drawLine(0, 20, 320, 20, ST77XX_WHITE);
    }

    // Power Draw // Battery Voltage // Battery Amps draw
    tft.setCursor(3, 28); tft.printf("W: %5.0f", batteryVoltage * batteryCurrent);
    tft.setCursor(3, 47); tft.printf("V: %5.2f", batteryVoltage);
    tft.setCursor(3, 66); tft.printf("A: %5.2f", batteryCurrent);
    tft.setCursor(3, 85); tft.printf("Pot:%3.0f%%", map(potThrottleVoltage, 0.3f, 3.3f, 0.0f, 100.0f));
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
    // tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    tft.setCursor(125, 147); tft.printf("Gear:  %d", selected_gear);
    tft.setCursor(125, 166); tft.printf("Power: %d", selected_power_mode);
    // tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    tft.setTextSize(1);
    // Uptime
    if (printUptime)  {
        tft.setCursor(3, 205);
        tft.printf("Uptime: %2dd %2dh %2dm %2ds\n", daysSinceBoot, hoursSinceBoot, minutesSinceBoot, secondsSinceBoot);
    }


    if (firsttime_draw) {
        // Bottom Divider
        tft.drawLine(0, 215, 320, 215, ST77XX_WHITE);
    }

    // Odometer
    tft.setTextSize(2);
    tft.setCursor(3, 220);
    tft.printf("O: %.1f", odometer);

    // Trip
    tft.setCursor(220, 220);
    tft.printf("T: %.1f", trip);


    if (printDebugStuff) {
        tft.setTextSize(1);
        sprintf(text2, "\nExecution time:"
                        "\n core0: %lu us   "
                        "\n core1: %lu us   \n",
                        timeCore0, timeCore1);
        tft.setCursor(160, 150);
        tft.println(text2);
    }

    if (firsttime_draw) {
        firsttime_draw = 0; // should be 0
    }
}

void loop2 (void* pvParameters) {
    while (1) {
        if (core1loopcount == 0)
            timeStartCore1 = timer_u32();

        // Reset temporary values
        _batteryVoltage = 0;
        _batteryCurrent = 0;
        _potThrottleVoltage = 0;

        for (int i = 0; i < 5; i++) {
            _batteryVoltage += adc1_get_raw(pinBatteryVoltage); // PIN 36/VP
            _batteryCurrent += adc1_get_raw(pinBatteryCurrent); // PIN 39/VN
            _potThrottleVoltage += adc1_get_raw(pinPotThrottle);
        }
        _batteryVoltage = _batteryVoltage / 5;
        _batteryCurrent = _batteryCurrent / 5;
        _potThrottleVoltage = _potThrottleVoltage / 5;

        // BATTERY VOLTAGE
        batteryVoltage = (30.265f *
            BatVoltageCorrection.correctInput(
                BatVoltageMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _batteryVoltage
                )
            )
        ); //29.839

        // BATTERY CURRENT
        batteryCurrent = (
            BatCurrentCorrection.correctInput(
                BatCurrentMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _batteryCurrent
                )
            )
        );

        // Throttle Voltage
        potThrottleVoltage = (
            PotThrottleCorrection.correctInput(
                PotThrottleMovingAverage.moveAverage(
                    ((float)3.3/(float)4095) * _potThrottleVoltage
                )
            )
        );

        // for now, directly map voltage from the throttle to the pot input pin of VESC
        dacWrite(pinOutToVESC, map(potThrottleVoltage, 0, 3.3, 0, 255));

        if (batteryPercentageVoltageBased) {
            batteryPercentage = map(batteryVoltage, 34, 41, 0, 100);
        } else {
            // TODO: implement amphour based battery percentage
            batteryPercentage = 0;
        }

        if (core1loopcount < 200) {
            core1loopcount++;
        } else {
            core1loopcount = 0;
            timeCore1 = (timer_u32() - timeStartCore1) / 200;
        }
    }
}

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

    BatVoltageCorrection.offsetPoints = {
    // input, offset
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

    BatCurrentCorrection.offsetPoints = BatVoltageCorrection.offsetPoints;
    PotThrottleCorrection.offsetPoints = BatVoltageCorrection.offsetPoints;

    BatVoltageCorrection.smoothingFactor = 1;
    BatVoltageMovingAverage.smoothingFactor = 0.01;

    BatCurrentCorrection.smoothingFactor = 1;
    BatCurrentMovingAverage.smoothingFactor = 0.01;

    PotThrottleCorrection.smoothingFactor = 1;
    PotThrottleMovingAverage.smoothingFactor = 0.5;


    // Configure ADC width (resolution)
    adc1_config_width(ADC_WIDTH);
    // Configure ADC channel and attenuation
    adc1_config_channel_atten(pinBatteryVoltage, ADC_ATTEN); // PIN 36/VP
    adc1_config_channel_atten(pinBatteryCurrent, ADC_ATTEN); // PIN 39/VN
    adc1_config_channel_atten(pinPotThrottle,    ADC_ATTEN); // PIN 34

    pinMode( pinOutToVESC, OUTPUT);
    dacWrite(pinOutToVESC, 0);

    pinMode(     pinTFTbacklight, OUTPUT); // Backlight of TFT
    digitalWrite(pinTFTbacklight, HIGH); // Turn on backlight

    ledcAttach(pinRotor, 5000, 8);
    ledcWrite( pinRotor, 100);



    // setup ebike namespace
    preferences.begin("ebike", false);
    // retrieve values
    odometer = preferences.getFloat("odometer", -1);
    trip     = preferences.getFloat("trip", -1);



    xTaskCreatePinnedToCore (
    loop2,     // Function to implement the task
    "loop2",   // Name of the task
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


            if (readString.contains("printDebug="))
            {
                printDebugStuff = getValueFromString("printDebug", readString);
                redrawScreen();
            }

            if (readString.contains("printUptime="))
            {
                printUptime = getValueFromString("printUptime", readString);
                redrawScreen();
            }

            // if (readString.contains("save"))
            // {
            //     clockMinutes = getValueFromString("clockMinutes", readString);
            // }

            readString="";
        }


        totalSecondsSinceBoot += ((float)timeCore0 / 1000000);
        secondsSinceBoot = (int)(totalSecondsSinceBoot) % 60;
        minutesSinceBoot = (int)(totalSecondsSinceBoot / 60) % 60;
        hoursSinceBoot =   (int)(totalSecondsSinceBoot / 60 / 60) % 24;
        daysSinceBoot =    totalSecondsSinceBoot / 60 / 60 / 24;


        // if ((timeEndDisplay - timeStartDisplay) > 66) { // 15Hz
        printDisplay();


          // totalSecondsSinceBoot = esp_timer_get_time() / 1000000; // +353000 = 4d 2h 3m 20s


          // Serial.printf("Uptime: %02d:%02d:%02d:%02d\n", daysSinceBoot, hoursSinceBoot, minutesSinceBoot, secondsSinceBoot);


          // timeStartDisplay = millis();
        // }
        // timeEndDisplay = millis();

        if (core0loopcount < 100) {
            core0loopcount++;
        } else {
            core0loopcount = 0;
        }
            timeCore0 = (timer_u32() - timeStartCore0);

    }
}
