#include "rollingRangeEstimation.hpp"

void RollingRangeEstimation::addDeltaDistance(double _currentDistanceKm) {
    currentDistanceKm += _currentDistanceKm;
}

void RollingRangeEstimation::addDeltaWhUsed(double _currentWhUsed) {
    currentWhUsed += _currentWhUsed;
}

void RollingRangeEstimation::loop(double remainingEnergyWh) {
    double tmp_currentDistanceKm = currentDistanceKm;
    double tmp_currentWhUsed = currentWhUsed;

    if (tmp_currentDistanceKm >= SEGMENT_LENGTH_KM) {

        addValueToArray(SEGMENTS, distanceSegment, tmp_currentDistanceKm);
        addValueToArray(SEGMENTS, whUsedSegment, tmp_currentWhUsed);

        currentDistanceKm -= tmp_currentDistanceKm;
        currentWhUsed -= tmp_currentWhUsed;

        sum.distance = 0.0;
        sum.whUsed = 0.0;

        for (int i = 0; i < SEGMENTS; i++) {
            sum.distance += distanceSegment[i];
            sum.whUsed += whUsedSegment[i];
        }

        sum.whPerKm = sum.whUsed / sum.distance;
        sum.range = remainingEnergyWh / sum.whPerKm;
    }
}

double RollingRangeEstimation::getEstimation() {
    return sum.range;
}

void RollingRangeEstimation::addValueToArray(int SIZE, double arr[], double newVal) {
    // Shift all values to the left
    for (int i = 0; i < SIZE - 1; ++i) {
        arr[i] = arr[i + 1];
    }
    // Add new value to the end
    arr[SIZE - 1] = newVal;
}