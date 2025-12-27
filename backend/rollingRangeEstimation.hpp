#include <iostream>
#include <iomanip>
#include <algorithm>

class RollingRangeEstimation {
private:
    static const int SEGMENTS = 100;
    double SEGMENT_LENGTH_KM = 0.02;

    double distanceSegment[SEGMENTS];
    double whUsedSegment[SEGMENTS];

    struct {
        double distance;
        double whUsed;
        double whPerKm;
        double range;
    } sum;

    double currentDistanceKm = 0.0;
    double currentWhUsed = 0.0;

public:
    void addDeltaDistance(double _currentDistanceKm);

    void addDeltaWhUsed(double _currentWhUsed);

    void loop(double remainingEnergyWh);

    double getEstimation();

    void addValueToArray(int SIZE, double arr[], double newVal);
};
