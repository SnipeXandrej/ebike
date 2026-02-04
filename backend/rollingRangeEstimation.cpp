#include "rollingRangeEstimation.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <mutex>

std::mutex mtx;

void RollingRangeEstimation::addDeltaDistance(double _currentDistanceKm) {
    mtx.lock();
    currentDistanceKm += _currentDistanceKm;
    mtx.unlock();
}

void RollingRangeEstimation::addDeltaWhUsed(double _currentWhUsed) {
    mtx.lock();
    currentWhUsed += _currentWhUsed;
    mtx.unlock();
}

void RollingRangeEstimation::loop(double remainingEnergyWh) {
    mtx.lock();
    double rollOverKilometers = NUM_OF_KILOMETERS + (NUM_OF_KILOMETERS * (PERCENT/100.0));
    double ratio = rollOverKilometers / NUM_OF_KILOMETERS;

    if (currentDistanceKm >= rollOverKilometers) {
        currentDistanceKm /= ratio;
        currentWhUsed /= ratio;
    }

    whPerKm = currentWhUsed / currentDistanceKm;
    range = remainingEnergyWh / whPerKm;

    mtx.unlock();
}

double RollingRangeEstimation::getRange() {
    return range;
}

double RollingRangeEstimation::getWhPerKm() {
    return whPerKm;
}