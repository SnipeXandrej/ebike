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

class Speedometer_Wheel {
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


static auto begin1 = std::chrono::steady_clock::now();
static auto end1 = std::chrono::steady_clock::now();

static int wheel_diameter_mm1;
static int wheel_circumference_mm1;
static int time_between_interrupt_duration_us1;

static double erpm_divider;
static double gear_ratio;
static double erpm_counter = 0;
static int duration_2;

class Speedometer_Motor {
public:
    static void myInterrupt() {
        end1 = std::chrono::steady_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end1 - begin1).count();

        // avoid spurious interrupts
        if (duration >= time_between_interrupt_duration_us1) {

            std::cout << "ERPM_Counter " << erpm_counter << "\n";
            if (erpm_counter < erpm_divider) {
                erpm_counter++;
                duration_2 += duration;
            } else {
                erpm_counter = 0;

                double duration_ms = (duration_2 / erpm_divider) / 1000;
                double erps = 1000/duration_ms;
                double erpm = erps * 60;
                double rpm = erpm / erpm_divider;
                double rps = erps / erpm_divider;

//                double distance_travelled_mm = wheel_circumference_mm * (1000/((double)duration_2 / 1000));
                double distance_travelled_mm = wheel_circumference_mm * 1/(1 / rps);
                       distance_travelled_mm = distance_travelled_mm / gear_ratio;
                double distance_travelled_meters = distance_travelled_mm / 1000;
                double distance_travelled_in_km = distance_travelled_meters / 1000;
                double speed_in_kmh = (distance_travelled_in_km * 3600);

                duration_2 = 0;
                std::cout << "speed_in_kmh: " << speed_in_kmh << "km/h" << "\n";
                std::cout << "Motor RPM: " << rpm << "\n";
            }

            begin1 = std::chrono::steady_clock::now();
        }
    }

    void start(int ISR_PIN, int mode) {
        pinMode(ISR_PIN, INPUT);
        wiringPiISR(ISR_PIN, mode, myInterrupt);
        pullUpDnControl(ISR_PIN, PUD_UP);
    }

    void init(int WHEEL_DIAMETER_MM, int TIME_BETWEEN_INTERRUPTS_DURATION_US,
              int MOTOR_SPROCKET_TEETH, int WHEEL_SPROCKET_TEETH, double ERPM_DIVIDER) {
        wheel_diameter_mm1 = WHEEL_DIAMETER_MM;
        wheel_circumference_mm1 = wheel_diameter_mm1 * M_PI;
        time_between_interrupt_duration_us1 = TIME_BETWEEN_INTERRUPTS_DURATION_US;
        erpm_divider = ERPM_DIVIDER;
        gear_ratio = MOTOR_SPROCKET_TEETH / WHEEL_SPROCKET_TEETH;
    }
};

#endif
