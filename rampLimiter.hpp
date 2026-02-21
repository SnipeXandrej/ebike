#include "timer.hpp"

class RampLimiter {
public:
    double getValue(double targetValue, double rampValueOverTime, double rampTimeMs_up, double rampTimeMs_down);

private:
    Timer timer;
    double output = 0;
};

