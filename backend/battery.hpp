#pragma once

// #include "map.hpp"
// #include "rampLimiter.hpp"
#include "commonUtils.hpp"

class Battery {
private:
    // ThrottleMap socVoltageMap;
    // RampLimiter voltageRampLimiter;
    // TODO: adjust via TOML
    const float cell_nominal_voltage = 3.65;
    const float cell_maximum_voltage = 4.2;
    const float cell_minimum_voltage = 3.0;
    const float cell_critical_minimum_voltage = 2.75;
    const float cell_series = 20;
    const float cell_user_maximum_voltage = 4.0;
    const float cell_user_minimum_voltage = 3.3;

    const float idleMaxDischargingCurrent = 0.5;
    const float idleMaxChargingCurrent = 0.35;
    const double voltageSettlingTimeMs = 600 * 1000; // 20 minutes
    const double accumulatedAmpHoursThreshold = 8; // number of amphours needed to proceed with calculating the estimated capacity

    double savedVoltagePercentage;

    double ampHoursFullyCharged;
    double ampHoursFullyChargedWhenNew;

    double ampHoursFullyCharged_tmp;

    double ampHoursUsedInElapsedTime;
    double wattHoursUsedInElapsedTime;

    double ampHoursEstimatedFullyCharged;

    double ampHoursUsedLifetime;

    double _percentage = 0.0;

    double percentageVoltageBased;

    double cycleCount = 0.0;

    const char* estimateStatusString;

public:
    void setup();

    void processLoop(double voltage_, double current_, bool batteryPercentageVoltageBased);

    double getAmpHoursFullyChargedWhenNew();
    void   setAmpHoursFullyChargedWhenNew(double newValue);

    double getAmpHoursFullyCharged();
    void   setAmpHoursFullyCharged(double newValue);

    double getAmpHoursUsedLifetime();
    void   setAmpHoursUsedLifetime(double newValue);
    double getWattHoursUsedInPreviousLoopCycle();

    double getAmpHoursUsed();
    void   setAmpHoursUsed(double newValue);

    double getCycleCount();
    void   setCycleCount(double newValue);

    // precalculated
    float voltage_nominal = cell_nominal_voltage * cell_series;
    float voltage_min = cell_minimum_voltage * cell_series;
    float voltage_max = cell_maximum_voltage * cell_series;
    float voltage_absolute_min = cell_critical_minimum_voltage * cell_series;
    float amphours_min_voltage = cell_user_minimum_voltage * cell_series;
    float amphours_max_voltage = cell_user_maximum_voltage * cell_series;

    double current;
    double voltage;
    double watts;

    double percentage = 100.0;

    double wattHoursUsed; // just an approximation
    double wattHoursFullyDischarged; // just an approximation
    double wattHoursRemaining; // just an approximation // calculated at runtime

    bool charging = false;
};
