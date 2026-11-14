#pragma once

class RampLimiter {
public:
    double getValue(double targetValue, double rampValueOverTime, double rampTimeMs_up, double rampTimeMs_down, double dt_ms);
    double getValue();

private:
    double output = 0;
};

