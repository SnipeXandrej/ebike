#include <iostream>
#include <wiringPi.h>
#include <mcp3004.h>
#include "temperature.h"
#include <thread>
#include <chrono>

using namespace std;

#define BASE 100
#define SPI_CHAN 0

int main()
{
    cout << "Hello World!" << endl;

    // setup mcp3004 and GPIO
    mcp3004Setup(BASE,SPI_CHAN);
    wiringPiSetupGpio(); // uses the BCM GPIO numbering

    // Temperature readout setup
    Temperature* temp1 = new Temperature();
    temp1->init(2200, 3930, 6800);

    Temperature* temp2 = new Temperature();
    temp2->init(2200, 3930, 6920);

    // calculate temperature
    while(1) {
        cout << "temp1: " << temp1->calculateTemp(analogRead(BASE+0)) << "\n";
        cout << "temp2: " << temp2->calculateTemp(analogRead(BASE+1)) << "\n";
        this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    return 0;
}
