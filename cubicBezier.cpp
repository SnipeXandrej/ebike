#include "cubicBezier.hpp"

float cubicBezier1D(float t, float p0, float p1, float p2, float p3) {
    float u = 1 - t;
    return u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t * p3;
}

float findTForX(float x, float x1, float x2, float epsilon = 0.001f) {
    float tLow = 0.0f;
    float tHigh = 1.0f;
    float t = 0.5f;

    while (tHigh - tLow > epsilon) {
        float xEstimate = cubicBezier1D(t, 0.0f, x1, x2, 1.0f);
        if (xEstimate < x) {
            tLow = t;
        } else {
            tHigh = t;
        }
        t = (tLow + tHigh) / 2.0f;
    }

    return t;
}

// map input x (0 to 100) to a cubic Bezier output
float mapToCubicBezier(float x, float x1, float y1, float x2, float y2) {
    float normalizedX = x / 100.0f;

    float t = findTForX(normalizedX, x1, x2);

    return cubicBezier1D(t, 0.0f, y1, y2, 1.0f) * 100.0f;
}