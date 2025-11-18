#include <iostream>
#include <vector>
#include <algorithm>

// throttle curve
struct Point {
    double in;
    double out;
};

class ThrottleMap {
public:
    void setCurve(const std::vector<Point>& setCurve) {
        curve = setCurve;

        std::sort(this->curve.begin(), this->curve.end(), [](const Point& a, const Point& b) { return a.in < b.in; });
    }

    double map(double input) const {
        if (input <= curve.front().in) return curve.front().out;
        if (input >= curve.back().in) return curve.back().out;

        for (size_t i = 0; i < curve.size() - 1; ++i) {
            if (input >= curve[i].in && input <= curve[i + 1].in) {
                double t = (input - curve[i].in) / (curve[i + 1].in - curve[i].in);
                // Linear interpolation
                return curve[i].out + t * (curve[i + 1].out - curve[i].out);
            }
        }
        return curve.back().out;
    }

private:
    std::vector<Point> curve;
};