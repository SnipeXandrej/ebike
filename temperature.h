#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <cmath>

class Temperature {
public:
    int RT0_1, B_1, R_1, R_2;
    double T0 = 25 + 273.15;
    void init(int thermistor_R, int thermistor_B, int resistor_R1) {
        RT0_1 = thermistor_R; //Ω
        B_1   = thermistor_B; //Beta of thermistor
        R_1   = resistor_R1;  //R=1,5KΩ
    }

    double VRT_1, VR_1, RT_1, ln_1, TX_1, Temperature;
    double VCC = 3.3;
    float temp;
    double calculateTemp(double input) {
        VRT_1 = input;                       //Acquisition of analog value of VRT
        VRT_1 = (VCC / 1023) * (1023 - VRT_1);  //VRT = (3.3 / 1023) * VRT; //Conversion to voltage
        VR_1 = VCC - VRT_1;
        RT_1 = VRT_1 / (VR_1 / R_1);              //Resistance of RT
        temp = (RT_1 / RT0_1);
        ln_1 = log(temp);
        TX_1 = (1 / ((ln_1 / B_1) + (1 / T0))); //Temperature from thermistor
        Temperature = TX_1 - 273.15;                   //Conversion to Celsius

        return Temperature;
    }

    ~Temperature();
};

#endif
