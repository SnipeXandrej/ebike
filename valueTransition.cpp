#include "valueTransition.hpp"

void ValueTransition::start() {
    timer.start();
}

double ValueTransition::getValueDifference(double fromValue, double toValue, double inMilliseconds) {
    timer.end();

    double timePassed = timer.getTime_ms();
    if (timePassed < 0.0) {
        timePassed = 0.0;
    }

    if (timePassed > inMilliseconds) {
        timePassed = inMilliseconds;
    }

    // linear interpolation
    float slope = (toValue - fromValue) / inMilliseconds;
    double output = toValue + slope * (timePassed - inMilliseconds);

    if (output != output) {
        output = 0.0;
    }

    return output;
}
