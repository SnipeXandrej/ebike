class RollingRangeEstimation {
private:
    const double NUM_OF_KILOMETERS = 2.0;
    const double PERCENT = 10.0;

    double currentDistanceKm = 0.0;
    double currentWhUsed = 0.0;

    double range;
    double whPerKm;

public:
    void addDeltaDistance(double _currentDistanceKm);
    void addDeltaWhUsed(double _currentWhUsed);
    void loop(double remainingEnergyWh);

    double getRange();
    double getWhPerKm();
};
