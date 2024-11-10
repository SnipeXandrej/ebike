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

    // setup mcp3004
    mcp3004Setup(BASE,SPI_CHAN);

    // Temperature readout setup
    Temperature* temp = new Temperature();
    temp->init(2200, 3930, 6800, 6920);

    // calculate temperature
    while(1) {
        cout << "Temperature: " << temp->calculateTemp(analogRead(BASE+0)) << "\n";
        this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    return 0;
}
