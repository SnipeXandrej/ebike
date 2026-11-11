#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <stdexcept>

// throttle curve
struct Point {
    double in;
    double out;
};

class ThrottleMap {
public:
    void setCurve(const std::vector<Point>& setCurve);
    double map(double input) const;

private:
    std::vector<Point> curve;
};
