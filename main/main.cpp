#include <stdio.h>
#include "Arduino.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include "inputOffset.h"
#include "fastGPIO.h"

bool firsttime = 1;


#define TFT_CS         5
#define TFT_RST        -1
#define TFT_DC         32
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int AnalogOut_temp;
float AnalogOut;
float correctedAnalogOut, correctedAnalogOut_temp;
InputOffset ADCcorrection;
InputOffset BatVoltageCorrection;

MovingAverage BatVoltageADCMovingAverage;

unsigned long timeStartCore1 = 0;
unsigned long timeEndCore1 = 0;
unsigned long timeCore1 = 0;

unsigned long timeStartCore0 = 0;
unsigned long timeEndCore0 = 0;
unsigned long timeCore0 = 0;

unsigned long timeStartDisplay = 0;
unsigned long timeEndDisplay = 0;
unsigned long timeDisplay = 0;

char text[128];
char text2[128];

// #include "driver/adc.h"
// #include "esp_adc_cal.h"
// #define ADC_CHANNEL    ADC1_CHANNEL_6  // GPIO34
// #define ADC_ATTEN      ADC_ATTEN_DB_11  // Allows reading up to 3.3V
// #define ADC_WIDTH      ADC_WIDTH_BIT_12 // 12-bit resolution

// Enabling C++ compile
extern "C" { void app_main(); }

void printDebug() {
        // tft.fillScreen(ST77XX_BLACK);
        sprintf(text, "RAW:       %.04fV"
              "\ncorrected_temp: %.04lf"
              "\nCorrected: %.04fV"
              "\n"
              "\nExecution time:"
              "\n core0: %lums   "
              "\n core1: %luus   \n",
              AnalogOut, correctedAnalogOut_temp, correctedAnalogOut, timeCore0, timeCore1);

        tft.setTextSize(2);
        tft.setCursor(0, 0);
        tft.println(text);
}

void printDisplay() {
      if (firsttime) {
        // Clock
        tft.setTextSize(2);
        tft.setCursor(3, 3); tft.println("ESP32 :)");

        // Battery
        tft.setCursor(268, 3); tft.println("100%");

        // Divider
        tft.drawLine(0, 20, 320, 20, ST77XX_WHITE);

        // Power Draw
        tft.setCursor(3, 48); tft.println("P: 127W");

        // consumption over last 1km
        tft.setCursor(180, 48); tft.println("3.7 Wh/km");

        // Speed
        tft.setTextSize(5);
        tft.setCursor(112, 120); tft.println("10");
        tft.setCursor(205, 172);
        tft.setTextSize(2); tft.println("km/h");

        // Odometer
        tft.setCursor(3, 220);
        tft.println("O: 1000.4");

        // Trip
        tft.setCursor(220, 220);
        tft.println("T: 370.0");

        firsttime = 1; // should be 0
      }

        tft.setTextSize(1);
        sprintf(text2, "\nExecution time:"
                      "\n core0: %lums   "
                      "\n core1: %luus   \n",
                      timeCore0, timeCore1);
        tft.setCursor(160, 120);
        tft.println(text2);
}

void loop2 (void* pvParameters) {
  while (1) {
    timeStartCore1 = micros();
    for (int i = 0; i < 10; i++) {
        // AnalogOut += adc1_get_raw(ADC_CHANNEL);
        AnalogOut_temp += analogRead(36);
    }
    AnalogOut_temp = AnalogOut_temp / 10;

    AnalogOut = BatVoltageADCMovingAverage.moveAverage( ((float)3.3/(float)4095) * AnalogOut_temp );
    correctedAnalogOut_temp = BatVoltageCorrection.correctInput(AnalogOut);

    correctedAnalogOut = (30.265f * correctedAnalogOut_temp); //29.839

    AnalogOut_temp = 0;

    timeEndCore1 = micros();
    timeCore1 = timeEndCore1 - timeStartCore1;
  }
}

void app_main(void)
{
     initArduino();
     Serial.begin(9600);
     Serial.println("Serial jeej");

     tft.init(240, 320);           // Init ST7789 320x240
     tft.setRotation(3);
     // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
     // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
     // may end up with a black screen some times, or all the time.
     tft.setSPISpeed(40000000);
     tft.invertDisplay(false);
     tft.fillScreen(ST77XX_BLACK);
     tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
     tft.setCursor(0, 0);

     xTaskCreatePinnedToCore (
       loop2,     // Function to implement the task
       "loop2",   // Name of the task
       10000,      // Stack size in bytes
       NULL,      // Task input parameter
       0,         // Priority of the task
       NULL,      // Task handle.
       1          // Core where the task should run
     );

       // // Configure ADC width (resolution)
       // adc1_config_width(ADC_WIDTH);

       // // Configure ADC channel and attenuation
       // adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

     ADCcorrection.offsetPoints = {
       // input, offset
       {0, 0},
       {60, 173}, // 10 OK
       {190, 190}, // 20 OK
       {330, 196}, // 30 OK
       {485, 200}, // 40 OK
       {630, 200}, // 50 OK
       {775, 211}, // 60 OK
       {915, 217}, // 70 OK
       {1060, 216}, // 80 OK
       {1202, 216}, // 90 OK
       {1355, 223}, // 100 OK
       {1495, 234}, // 110 OK
       {1640, 231}, // 120 OK
       {1820, 238}, // 130 OK
       {1960, 244}, // 140 OK
       {2105, 243}, // 150 OK
       {2255, 259}, // 160 OK
       {2397, 259}, // 170 OK
       {2540, 266}, // 180 OK
       {2695, 262}, // 190 OK
       {2838, 264}, // 200 OK
       {3005, 238}, // 210 OK
       {3197, 200}, // 220 OK
       {3426, 120}, // 230 OK
       {3665, 28}, // 240 OK
       {3933, -96}, // 250 OK
       {4072, -160}, // 255 OK
       {4095, -160}
   };

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
     BatVoltageCorrection.smoothingFactor = 1;
     BatVoltageADCMovingAverage.smoothingFactor = 0.01;

     pinMode(36, INPUT);   // 36 ADC
     pinMode(25, OUTPUT);  // D25 DAC

     pinMode(2, OUTPUT); // Backlight of TFT
     digitalWrite(2, HIGH); // Turn on backlight

     dacWrite(25, 37);

    while(1) {
        timeStartCore0 = millis();

        // if ((timeEndDisplay - timeStartDisplay) > 66) { // 15Hz
          printDebug();
          // timeStartDisplay = millis();
        // }
        // timeEndDisplay = millis();


        timeEndCore0 = millis();
        timeCore0 = timeEndCore0 - timeStartCore0;
    }
}
