#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <wiringPi.h>
#include <chrono>
#include <cmath>

static auto begin = std::chrono::steady_clock::now();
static auto end = std::chrono::steady_clock::now();

static int wheel_diameter_mm;
static int wheel_circumference_mm;
static int time_between_interrupt_duration_us;

class Speedometer {
public:
    static void myInterrupt() {
        end = std::chrono::steady_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        // avoid spurious interrupts
        if (duration >= time_between_interrupt_duration_us) {
            double duration_ms = duration / 1000;
            double distance_travelled_mm = wheel_circumference_mm * (1000/duration_ms);
            double distance_travelled_meters = distance_travelled_mm / 1000;
            double distance_travelled_in_km = distance_travelled_meters / 1000;
            double speed_in_kmh = (distance_travelled_in_km * 3600);

            std::cout << "speed_in_kmh: " << speed_in_kmh << "km/h" << "\n";

            begin = std::chrono::steady_clock::now();
        }
    }

    void start(int ISR_PIN, int mode) {
        pinMode(ISR_PIN, INPUT);
        wiringPiISR(ISR_PIN, mode, myInterrupt);
        pullUpDnControl(ISR_PIN, PUD_UP);
    }

    void init(int WHEEL_DIAMETER_MM, int TIME_BETWEEN_INTERRUPTS_DURATION_US) {
        wheel_diameter_mm = WHEEL_DIAMETER_MM;
        wheel_circumference_mm = wheel_diameter_mm * M_PI;
        time_between_interrupt_duration_us = TIME_BETWEEN_INTERRUPTS_DURATION_US;
    }
};

#endif
