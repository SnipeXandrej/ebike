#include <iostream>
#include <thread>
#include <chrono>
#include <wiringPi.h>
#include <mcp3004.h>
#include "temperature.h"
#include "speedometer.h"

using namespace std;

#define BASE 100
#define SPI_CHAN 0

int main()
{
    cout << "Hello!" << endl;

    // setup mcp3004 and GPIO
    mcp3004Setup(BASE,SPI_CHAN);
    wiringPiSetupGpio(); // uses the BCM GPIO numbering

    // Speedometer setup and start
    Speedometer_Wheel *speedometer_front_wheel = new Speedometer_Wheel;
    speedometer_front_wheel->init(630, 71.26*1000); // with 71.26ms the limit is 100km/h with the 630mm wheel diameter
    speedometer_front_wheel->start(20, INT_EDGE_FALLING);

    Speedometer_Motor *speedometer_motor = new Speedometer_Motor;
    speedometer_motor->init(630, 2.5*1000, 60, 10, 6); // with 2.5ms the limit is 24k ERPM, or 4000RPM
    speedometer_motor->start(21, INT_EDGE_FALLING);

    // Temperature readout setup
    Temperature* temp1 = new Temperature();
    temp1->init(2200, 3930, 6800);

    Temperature* temp2 = new Temperature();
    temp2->init(2200, 3930, 6920);

    // calculate and print temperature
    while(1) {
//        cout << "temp1: " << temp1->calculateTemp(analogRead(BASE+0)) << "\n";
//        cout << "temp2: " << temp2->calculateTemp(analogRead(BASE+1)) << "\n";
        this_thread::sleep_for(std::chrono::milliseconds(1000));
//        cout << "var1: " << speedometer_front_wheel->
    }

    return 0;
}
