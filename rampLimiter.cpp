#include "rampLimiter.hpp"

double RampLimiter::getValue(double targetValue, double rampValueOverTime, double rampTimeMs_up, double rampTimeMs_down) {
    timer.end();

    double timeDivider_up = rampTimeMs_up / timer.getTime_ms();
    double rampValueDivided_up = rampValueOverTime / timeDivider_up;

    if (output < targetValue) {
        output += rampValueDivided_up;
        if (output > targetValue)
            output = targetValue;
    }

    double timeDivider_down = rampTimeMs_down / timer.getTime_ms();
    double rampValueDivided_down = rampValueOverTime / timeDivider_down;

    if (output > targetValue) {
        output -= rampValueDivided_down;
        if (output < targetValue)
            output = targetValue;
    }

    timer.start();
    return output;
}

double RampLimiter::getValue() {
    return output;
}
