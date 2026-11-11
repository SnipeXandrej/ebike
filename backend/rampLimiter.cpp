#include "rampLimiter.hpp"

double RampLimiter::getValue(double targetValue, double rampValueOverTime, double rampTimeMs_up, double rampTimeMs_down, double dt) {
    double timeDivider_up = rampTimeMs_up / (dt * 1000.0);
    double rampValueDivided_up = rampValueOverTime / timeDivider_up;

    if (output < targetValue) {
        output += rampValueDivided_up;
        if (output > targetValue)
            output = targetValue;
    }

    double timeDivider_down = rampTimeMs_down / (dt * 1000.0);
    double rampValueDivided_down = rampValueOverTime / timeDivider_down;

    if (output > targetValue) {
        output -= rampValueDivided_down;
        if (output < targetValue)
            output = targetValue;
    }

    return output;
}

double RampLimiter::getValue() {
    return output;
}
