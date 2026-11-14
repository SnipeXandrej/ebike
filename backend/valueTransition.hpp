#include "timer.hpp"

class ValueTransition {
public:
    void start();
    double getValueDifference(double fromValue, double toValue, double inMilliseconds);

    Timer timer;
};