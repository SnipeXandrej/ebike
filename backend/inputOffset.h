class MovingAverage {
private:
    float previousInput;
    float Input;
    bool done = 0;

    float smoothValue(float newValue, float previousValue, float smoothingFactor) {
        return previousValue + smoothingFactor * (newValue - previousValue);
    }

public:
    float smoothingFactor = 1;
    float output;

    void initInput(float input) {
        if (!done) {
            previousInput = input;
            Input = input;
            done = 1;
        }
    }

    void setInput(float input) {
        previousInput = input;
        Input = input;
    }

    float moveAverage(float input) {
        Input = smoothValue(input, previousInput, smoothingFactor);
        previousInput = Input;

        output = Input;

        return Input;
    }
};

class InputOffset {
private:
    // Structure to hold input readings and offsets
    struct OffsetPoint {
        float inputVoltage;  // input value
        float offset;        // Offset corresponding to the input
    };

    // Linear interpolation function
    float interpolateOffset(const std::vector<OffsetPoint>& points, float input) {
        // Find the two points between which the input voltage lies
        for (size_t i = 0; i < points.size() - 1; ++i) {
            if (input >= points[i].inputVoltage && input <= points[i + 1].inputVoltage) {
                float t = (input - points[i].inputVoltage) / (points[i + 1].inputVoltage - points[i].inputVoltage);
                return points[i].offset + t * (points[i + 1].offset - points[i].offset);
            }
        }

        // Extrapolate if outside the known range
        if (input < points.front().inputVoltage) {
            return points.front().offset;
        }
        if (input > points.back().inputVoltage) {
            return points.back().offset;
        }

        return 0.0; // Default case (shouldn't reach here if input is valid)
    }

public:
    std::vector<OffsetPoint> offsetPoints;
    float smoothingFactor = 1; // Smoothing factor (0 = no smoothing, 1 = instant change)

    float correctInput(float input) {
        // Interpolate offset
        float offset = interpolateOffset(offsetPoints, input);

        // Compute and return adjusted RPM
        return input + offset;
    }
};
