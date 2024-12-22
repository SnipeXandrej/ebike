#ifndef GAUGERPM_H
#define GAUGERPM_H

#include <chrono>
#include <thread>
#include <wiringPi.h>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class GaugeRPM {
public:
    // Structure to hold gauge readings and offsets
    struct OffsetPoint {
        double rpm;       // RPM value
        double offset;    // Offset corresponding to the RPM
    };

    std::vector<OffsetPoint> offsetPoints = {
        // RPM, offset
        {300, 180},
        {500, 130},
        {1000, -80},
        {1500, -116},
        {2000, -123},
        {2500, -100},
        {3000, -80},
        {3500, -40},
        {4000, -50},
        {4500, -20},
        {5000, -50},
        {6000, 0}
    };

    // Linear interpolation function
    double interpolateOffset(const std::vector<OffsetPoint>& points, double gaugeRPM) {
        // Find the two points between which the gaugeRPM lies
        for (size_t i = 0; i < points.size() - 1; ++i) {
            if (gaugeRPM >= points[i].rpm && gaugeRPM <= points[i + 1].rpm) {
                double t = (gaugeRPM - points[i].rpm) / (points[i + 1].rpm - points[i].rpm);
                return points[i].offset + t * (points[i + 1].offset - points[i].offset);
            }
        }

        // Extrapolate if outside the known range
        if (gaugeRPM < points.front().rpm) {
            return points.front().offset;
        }
        if (gaugeRPM > points.back().rpm) {
            return points.back().offset;
        }

        return 0.0; // Default case (shouldn't reach here if input is valid)
    }

    // Smoothing function using a moving average
    double smoothValue(double newValue, double previousValue, double smoothingFactor) {
        return previousValue + smoothingFactor * (newValue - previousValue);
    }

    double smoothingFactor = 1; // Smoothing factor (0 = no smoothing, 1 = instant change)
    double previousAdjustedRPM = 0;

    void init(int pin) {
        pinMode(pin, OUTPUT);
    }

    int inputRPM;
    double frequency=171.6; //2500RPM
    int milliseconds=(1000000/frequency);

    void input() {
        while(1) {
               cin >> inputRPM;

               // Interpolate offset
               double offset = interpolateOffset(offsetPoints, inputRPM);

               // Compute adjusted RPM
               double adjustedRPM = inputRPM + offset;

               // Apply smoothing
               adjustedRPM = smoothValue(adjustedRPM, previousAdjustedRPM, smoothingFactor);

               // Store the smoothed value as previous for the next iteration
               previousAdjustedRPM = adjustedRPM;

               // Output the result
               std::cout << "Gauge RPM: " << inputRPM
                         << ", Offset: " << offset
                         << ", Adjusted RPM: " << adjustedRPM << std::endl;

               frequency = (((float)413/(float)6000) * adjustedRPM);
               cout << "frequency = " << frequency << "\n";

               milliseconds=(1000000/(frequency));
        }
    }

    void start() {
        while(1) {
            this_thread::sleep_for(std::chrono::microseconds(milliseconds));
            digitalWrite(14, 0);
            this_thread::sleep_for(std::chrono::microseconds(milliseconds));
            digitalWrite(14, 1);
        }
    }
};

#endif //GAUGERPM_H
