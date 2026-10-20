#include "battery.hpp"
#include "commonUtils.hpp"
#include <chrono>
#include <thread>
#include <format>

// this implementation requires the use of SoC vs Voltage values

void Battery::setup() {
    // my measurement of the battery pack
    socVoltageMap.setCurve({
        /* Voltage, Percentage */
        {3.01, 0},
        {3.195, 1.26},
        {3.331, 2.96},
        {3.4025, 4.66},
        {3.455, 11.47},
        {3.518, 18.28},
        {3.56, 25.09},
        {3.589, 31.91},
        {3.606, 38.71},
        {3.63, 45.52},
        {3.6625, 52.33},
        {3.726, 59.14},
        {3.7925, 65.95},
        {3.855, 72.76},
        {3.9225, 79.57},
        {3.995, 86.38},
        {4.071, 93.19},
        {4.16, 100}
    });

    // // https://www.ti.com/lit/an/sluaar3/sluaar3.pdf?ts=1789490245834
    // socVoltageMap.setCurve({
    //     {3.0, 0}, // {2.8, 0} <- Original
    //     {3.2, 10},
    //     {3.38, 20},
    //     {3.53, 30},
    //     {3.61, 40},
    //     {3.72, 50},
    //     {3.82, 60},
    //     {3.90, 70},
    //     {3.99, 80},
    //     {4.07, 90},
    //     {4.16, 100}
    // });

    // // https://zbotic.in/how-to-read-battery-state-of-charge-voltage-vs-actual-capacity/#why-voltage-misleads
    // socVoltageMap.setCurve({
    //     {3.20, 0},
    //     {3.49, 10},
    //     {3.61, 20},
    //     {3.67, 30},
    //     {3.71, 40},
    //     {3.74, 50},
    //     {3.78, 60},
    //     {3.83, 70},
    //     {3.96, 80},
    //     {4.08, 90},
    //     {4.20, 100}
    // });
}

double Battery::getAmpHoursFullyChargedWhenNew() {
    return ampHoursFullyChargedWhenNew;
};

void Battery::setAmpHoursFullyChargedWhenNew(double newValue) {
    ampHoursFullyChargedWhenNew = newValue ;
};

double Battery::getAmpHoursFullyCharged() {
    return ampHoursFullyCharged;
};

void Battery::setAmpHoursFullyCharged(double newValue) {
    ampHoursFullyCharged = newValue;
    ampHoursFullyCharged_tmp = newValue;
    ampHoursEstimatedFullyCharged = newValue;
}

double Battery::getAmpHoursUsedLifetime() {
    return ampHoursUsedLifetime;
};

void Battery::setAmpHoursUsedLifetime(double newValue) {
    ampHoursUsedLifetime = newValue;
};

double Battery::getWattHoursUsedInPreviousLoopCycle() {
    return wattHoursUsedInElapsedTime;
};

double Battery::getAmpHoursUsed() {
    return ampHoursFullyCharged * ((100.0 - _percentage) / 100.0);
};

void Battery::setAmpHoursUsed(double newValue) {
    // nothing
};

double Battery::getCycleCount() {
    return cycleCount;
};

void Battery::setCycleCount(double newValue) {
    cycleCount = newValue;
};

enum AMPHOUR_ESTIMATE {
    START = 0,
    START_TO_RUNNING = 4,
    RUNNING = 1,
    END = 2,
};

struct {
    int state = AMPHOUR_ESTIMATE::START;
    bool isVoltageBased = false;
    double startVoltagePercentage;
    double endVoltagePercentage;
    double accAmpHours;
    double accWattHours;
    double deltaPercentage;
    double estimatedCapacityAmpHours;
} AMPHOUR_ESTIMATE_T;

void Battery::processLoop(double voltage_, double current_, bool batteryPercentageVoltageBased) {
    static auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> usElapsed = std::chrono::high_resolution_clock::now() - t1;

    voltage = voltage_;
    current = current_;

    watts = voltage * current;
    wattHoursFullyDischarged = (cell_nominal_voltage * cell_series) * ampHoursFullyCharged;
    wattHoursRemaining = wattHoursFullyDischarged * (_percentage / 100.0);
    wattHoursUsed = wattHoursFullyDischarged - wattHoursRemaining;

    ampHoursUsedInElapsedTime = current / (1000000.0 / usElapsed.count()) / 3600.0;
    wattHoursUsedInElapsedTime = (current * voltage) / (1000000.0 / usElapsed.count()) / 3600.0;

    if ((wattHoursUsedInElapsedTime >= 20.0) || (wattHoursUsedInElapsedTime <= -20.0)) {
        wattHoursUsedInElapsedTime = 0.0;
    }

    if ((ampHoursUsedInElapsedTime >= 20.0) || (ampHoursUsedInElapsedTime <= -20.0)) {
        ampHoursUsedInElapsedTime = 0.0;
    }

    percentageVoltageBased = voltageRampLimiter.getValue(socVoltageMap.map(voltage / cell_series), 0.1, 5000, 5000);

    // combined percentage
    static auto timeVoltageSettledStart = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> timeVoltageSettledMsElapsed = std::chrono::high_resolution_clock::now() - timeVoltageSettledStart;

    if (current <= idleMaxDischargingCurrent && current >= -idleMaxChargingCurrent) {
        if (timeVoltageSettledMsElapsed.count() >= voltageSettlingTimeMs) {
            AMPHOUR_ESTIMATE_T.isVoltageBased = true;

            // if (/*something*/) {
            //     // disable idle
            //     AMPHOUR_ESTIMATE_T.isVoltageBased = false;
            //     timeVoltageSettledStart = std::chrono::high_resolution_clock::now();
            // }
        } else {
            AMPHOUR_ESTIMATE_T.isVoltageBased = false;
        }
    } else {
        AMPHOUR_ESTIMATE_T.isVoltageBased = false;
        timeVoltageSettledStart = std::chrono::high_resolution_clock::now();
    }

    static bool firstTime = true;
    static double accAmpHours = 0.0;
    static double accAmpHours_abs = 0.0;
    static double accPercentageChange = 0.0;
    static bool state_isVoltageBased = false;
    static double ampHoursLifetime_savedVoltagePercentage = 0.0;

    // - also does lifetime amphours counting
    // - when isVoltageBased=true save the percentageVoltageBased, when isVoltageBased=false calculate the amphour used
    //   from the percentage difference and add it to ampHoursUsedLifetime
    // - also if isVoltageBased=false, add the accAmpHours value to ampHoursUsedLifetime
    // - make sure to do this only once every isVoltageBased change
    // - edge-case... while in isVoltageBased=true, the amphours are counted by the difference when leaving
    //   and entering isVoltageBased=true, so if the current is low enough to not trigger non-idleness, the
    //   battery can be slowly charged/discharged below the set idle limits, that means in reality it will accumulate
    //   amphours, but those are not going to be counted
    //   - possible solution?: always count the lifetimeamphours and cycleCount, and only stop counting mentioned values when
    //     the battery is charging and the current is >-0.05 && <0.05
    //   - another possible solution: rename isVoltageBased to isIdle, then while its in idle, if the last saved voltage changes
    //     too much (maybe like 0.2V) the idle state will be set to false
    //   - another possible solution: rename isVoltageBased to isIdle, then while its in idle, if the last saved voltage changes
    //     too much (maybe like 0.2V) over the course of some time, the idle state will be set to false
    //   - the problem is that I dont want it to be dependent on the "charging" variable, and maybe I am just overthinking this...
    if (AMPHOUR_ESTIMATE_T.isVoltageBased) {
        if (!state_isVoltageBased) {
            state_isVoltageBased = true;

            // add accAmpHours before its reset
            ampHoursUsedLifetime += std::fabs(accAmpHours_abs);

            cycleCount += (std::fabs(accAmpHours_abs)/getAmpHoursFullyCharged()) / 2.0;

            ampHoursLifetime_savedVoltagePercentage = percentageVoltageBased;
        }

        accAmpHours = 0.0;
        accAmpHours_abs = 0.0;
        accPercentageChange = 0.0;
        savedVoltagePercentage = percentageVoltageBased;
    } else {
        if (firstTime) {
            savedVoltagePercentage = percentageVoltageBased;
            firstTime = false;
        }

        if (state_isVoltageBased) {
            state_isVoltageBased = false;

            double ampHoursAccumulatedDuringIdle = getAmpHoursFullyCharged() * ((ampHoursLifetime_savedVoltagePercentage - savedVoltagePercentage) / 100.0);
            ampHoursUsedLifetime += std::fabs(ampHoursAccumulatedDuringIdle);

            cycleCount += (std::fabs(ampHoursAccumulatedDuringIdle)/getAmpHoursFullyCharged()) / 2.0;
        }

        accAmpHours += ampHoursUsedInElapsedTime;
        accAmpHours_abs += std::fabs(ampHoursUsedInElapsedTime);
        accPercentageChange = (accAmpHours / getAmpHoursFullyCharged()) * 100.0;
    }

    _percentage = savedVoltagePercentage - accPercentageChange;
    static RampLimiter voltageTransition;
    static double _percentage2 = 0.0;

    // slowly adjust the percentage shown if theres a big difference between the voltagePercentage/_percentage hybrid
    // and just the voltagePercentage one
    if (!AMPHOUR_ESTIMATE_T.isVoltageBased) {
        if ((_percentage2 - _percentage) >= 0.1 || (_percentage - _percentage2) >= 0.1) {
            _percentage2 = voltageTransition.getValue(_percentage, 0.1, 20000, 20000);
        } else {
            _percentage2 = voltageTransition.getValue(_percentage, 0.1, 1, 1);
        }
    } else {
        _percentage2 = voltageTransition.getValue(_percentage, 0.1, 20000, 20000);
    }

    // AMPHOUR COUNTING WITH DATA ACCUMULATED DURING USAGE
    // only used for setting/changing the value of ampHoursFullyCharged
    switch(AMPHOUR_ESTIMATE_T.state) {
        case AMPHOUR_ESTIMATE::START: {
            estimateStatusString = "Waiting for voltage to settle"; // || current > -idleMaxDischargingCurrent
            estimateStatusString += std::format("\n wait {:.2f} second(s)", (voltageSettlingTimeMs - timeVoltageSettledMsElapsed.count()) / 1000.0);
            // if (current <= -idleMaxDischargingCurrent) break; // charging ||
            if (AMPHOUR_ESTIMATE_T.isVoltageBased) {
                AMPHOUR_ESTIMATE_T.startVoltagePercentage = percentageVoltageBased;
                estimateStatusString = "(IN START) 1. Ready to start collecting data";
                AMPHOUR_ESTIMATE_T.state = AMPHOUR_ESTIMATE::START_TO_RUNNING;
            }
            break;
        }
        case AMPHOUR_ESTIMATE::START_TO_RUNNING: {
            if (AMPHOUR_ESTIMATE_T.isVoltageBased) {
                AMPHOUR_ESTIMATE_T.startVoltagePercentage = percentageVoltageBased;
                estimateStatusString = "1. Ready to start collecting data";
            } else {
                AMPHOUR_ESTIMATE_T.accAmpHours = 0;
                AMPHOUR_ESTIMATE_T.accWattHours = 0;
                AMPHOUR_ESTIMATE_T.state = AMPHOUR_ESTIMATE::RUNNING;
            }
            break;
        }
        case AMPHOUR_ESTIMATE::RUNNING: {
            estimateStatusString = "2. Collecting data";

            AMPHOUR_ESTIMATE_T.accAmpHours += ampHoursUsedInElapsedTime;
            AMPHOUR_ESTIMATE_T.accWattHours += wattHoursUsedInElapsedTime; // TODO: NOT USED RIGHT NOW

            if (AMPHOUR_ESTIMATE_T.isVoltageBased) {
                if (AMPHOUR_ESTIMATE_T.accAmpHours < accumulatedAmpHoursThreshold) {
                    AMPHOUR_ESTIMATE_T.state = AMPHOUR_ESTIMATE::START;
                    estimateStatusString = "3. Not enough data accumulated";
                    break;
                };

                AMPHOUR_ESTIMATE_T.endVoltagePercentage = percentageVoltageBased;

                AMPHOUR_ESTIMATE_T.state = AMPHOUR_ESTIMATE::END;
            }
            break;
        }
        case AMPHOUR_ESTIMATE::END: {
            AMPHOUR_ESTIMATE_T.deltaPercentage = AMPHOUR_ESTIMATE_T.startVoltagePercentage - AMPHOUR_ESTIMATE_T.endVoltagePercentage;
            if (AMPHOUR_ESTIMATE_T.deltaPercentage < 0.0) {
                AMPHOUR_ESTIMATE_T.state = AMPHOUR_ESTIMATE::START;
                estimateStatusString = "4. delta percentage was in the negatives, failed";
                break;
            }

            AMPHOUR_ESTIMATE_T.estimatedCapacityAmpHours = AMPHOUR_ESTIMATE_T.accAmpHours / (AMPHOUR_ESTIMATE_T.deltaPercentage / 100.0);
            std::printf("Experimental estimated capacity: %f\n", AMPHOUR_ESTIMATE_T.estimatedCapacityAmpHours);
            std::printf("Experimental estimated capacity deltaPercentage: %f\n", AMPHOUR_ESTIMATE_T.deltaPercentage);
            std::printf("Experimental estimated capacity accAmpHours: %f\n", AMPHOUR_ESTIMATE_T.accAmpHours);

            ampHoursFullyCharged = ampHoursFullyCharged == 0.0
                                                ? AMPHOUR_ESTIMATE_T.estimatedCapacityAmpHours
                                                : (ampHoursFullyCharged + AMPHOUR_ESTIMATE_T.estimatedCapacityAmpHours) / 2.0;

            estimateStatusString = "(IN END) 5. New estimated capacity saved";
            AMPHOUR_ESTIMATE_T.state = AMPHOUR_ESTIMATE::START;
            break;
        }
    }

    // Battery charge tracking stuff
    if (batteryPercentageVoltageBased) {
        percentage = percentageVoltageBased;
    } else {
        percentage = _percentage2;
    }

    std::string _statusString = std::format(
        "   IsVoltageBased:                  {}\n"
        "   Percentage shown:                {:.3f} %\n"
        "   percentage Voltage Based:        {:.3f} %\n"
        "\n   AmpHours Estimated Used:         {:.3f} A\n"
        "   AmpHours Estimated Full:         {:.3f} A\n"
        "   Acc AmpHours During Usage:       {:.3f}\n"
        "   Acc AmpHours (abs) During Usage: {:.3f}\n"
        "   Acc Percentage During Usage:     {:.3f}\n"
        "   Collecting data status:          {}\n"
        "   Accumulated Amphours during usage:  {:.3f} Ah\n"
        "   Accumulated Watthours during usage: {:.3f} Wh\n"
        "   Estimated Capacity from test:       {:.3f} %\n"
        "   Delta Percentage:                   {:.3f} %\n"
        , AMPHOUR_ESTIMATE_T.isVoltageBased
        , percentage
        , percentageVoltageBased
        , getAmpHoursUsed()
        , getAmpHoursFullyCharged()
        , accAmpHours
        , accAmpHours_abs
        , accPercentageChange
        , estimateStatusString
        , AMPHOUR_ESTIMATE_T.accAmpHours
        , AMPHOUR_ESTIMATE_T.accWattHours
        , AMPHOUR_ESTIMATE_T.estimatedCapacityAmpHours
        , AMPHOUR_ESTIMATE_T.deltaPercentage
    );

    _statusString += std::format(
        "\n   idleMaxDischargingCurrent:    {} A\n"
        "   idleMaxChargingCurrent:       {} A\n"
        "   voltageSettlingTimeMs:        {:.1f}\n"
        "   accumulatedAmpHoursThreshold: {} Ah\n"
        ,idleMaxDischargingCurrent
        ,idleMaxChargingCurrent
        ,voltageSettlingTimeMs
        ,accumulatedAmpHoursThreshold
    );

    statusString.set(_statusString);

    t1 = std::chrono::high_resolution_clock::now();
}
