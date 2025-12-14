#include "imguiGestures.hpp"
#include "timer.hpp"
#include "other.hpp"
#include <print>

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

float c_x1 = 0.16f;
float c_y1 = 0.08f;
float c_x2 = 0.0f;
float c_y2 = 0.88f;

class ValueTransition {
public:
    void start() {
        timer.start();
    }

    double getValueDifference(double fromValue, double toValue, double inMilliseconds) {
        timer.end();

        double timePassed = timer.getTime_ms();
        double timeDivision = inMilliseconds / timePassed;

        double output = (toValue - fromValue) / timeDivision;

        if (output < fromValue) {
            output = fromValue;
        }

        if (output > toValue) {
            output = toValue;
        }

        if (output != output) {
            output = 0.0;
        }

        // std::print("output: {}\n", output);

        return output;
    }

private:
    Timer timer;
    // double fromValue;
    // double toValue;
    // double inMilliseconds;
};
ValueTransition valueTransition;

bool closeWindow = false;
bool continueGesture = false;
float animationMoveOffset;
float appSizeY = 0;
float appSizeX = 0;
float windowPosY = 0;
float windowPosYOnLetoff = 0;

ImVec2 windowSize = ImVec2(300, 250);
float gestureMaxY = 300;
float gestureThresholdY = 200;

enum {
    GESTURE_NOTHING = 0,
    GESTURE_START = 1,
    GESTURE_HIDE = 2,
    GESTURE_TOO_HIGH = 3,
    GESTURE_CLOSE = 4,
    GESTURE_MOVE_TO_HIGHEST = 5,
    GESTURE_CLOSING_ACTION = 6,
};

int gestureCase = GESTURE_NOTHING;

void ImGuiGesture::start() {
    appSizeY = ImGui::GetWindowSize().y;
    appSizeX = ImGui::GetWindowSize().x;

    bool isGestureWithinStartRegion = ((io.MousePos.y < appSizeY) && (io.MousePos.y > appSizeY - 55.0)) ? true : false;
    bool isHeaderWithinConstraints = (io.MousePos.y > (appSizeY - gestureMaxY)) && (io.MousePos.y < (appSizeY - gestureMaxY + 40.0))
                                  && (io.MousePos.x > (appSizeX/2.0 - (windowSize.x/2.0)) && io.MousePos.x < (appSizeX/2.0 + (windowSize.x/2.0))) ? true : false;

    switch (gestureCase) {
    case GESTURE_NOTHING:
        // hide it
        if (!continueGesture) {
            windowPosY = appSizeY;
        }

        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !continueGesture) {
            gestureCase = GESTURE_START;
        }

        if (continueGesture) {
            if (isHeaderWithinConstraints) {
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    gestureCase = GESTURE_CLOSING_ACTION;
                }
            }
        }
        break;

    case GESTURE_START:
        if (isGestureWithinStartRegion || continueGesture) {

            if (continueGesture == false) {
                continueGesture = true;
                animationMoveOffset = appSizeY - io.MousePos.y;
            }

            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                windowPosY = io.MousePos.y + animationMoveOffset;
            } else {
                valueTransition.start();
                windowPosYOnLetoff = windowPosY;

                // if the gesture went above the max height, move it down
                if ((appSizeY - windowPosY) > gestureMaxY) {
                    gestureCase = GESTURE_TOO_HIGH;
                    break;
                }

                // if the gesture didnt reach the threshold, close the window
                if ((appSizeY - windowPosY) < gestureThresholdY) {
                    gestureCase = GESTURE_CLOSE;
                    break;
                }

                gestureCase = GESTURE_MOVE_TO_HIGHEST;

            }
        }

        break;

    case GESTURE_HIDE:
        continueGesture = false;

        gestureCase = GESTURE_NOTHING;
        break;

    case GESTURE_TOO_HIGH:
        static float output;
        output += valueTransition.getValueDifference(0.0, 100.0, 300.0); // move down
        valueTransition.start();

        static float output_bezier;
        output_bezier = mapToCubicBezier(output, c_x1, c_y1, c_x2, c_y2) + 1.0;

        static float destination;
        destination = appSizeY - gestureMaxY;

        windowPosY = map_f(output_bezier, 0, 100, windowPosYOnLetoff, destination);

        if ((appSizeY - windowPosY) <= gestureMaxY) {
            windowPosY = destination;
            output = 0;
            gestureCase = GESTURE_NOTHING;
        }

        break;

    case GESTURE_CLOSE:
        output += valueTransition.getValueDifference(0.0, 100.0, 150.0); // move down
        valueTransition.start();

        output_bezier = mapToCubicBezier(output, c_x1, c_y1, c_x2, c_y2) + 1.0;

        destination = appSizeY;

        windowPosY = map_f(output_bezier, 0, 100, windowPosYOnLetoff, destination);

        if ((int)windowPosY >= (int)destination) {
            gestureCase = GESTURE_HIDE;
            output = 0;
        }

        break;

    case GESTURE_MOVE_TO_HIGHEST:
        output += valueTransition.getValueDifference(0.0, 100.0, 300.0); // move down
        valueTransition.start();

        output_bezier = mapToCubicBezier(output, c_x1, c_y1, c_x2, c_y2) + 1.0;

        destination = appSizeY - gestureMaxY;

        windowPosY = map_f_nochecks(output_bezier, 0, 100, windowPosYOnLetoff, destination);

        if ((appSizeY - windowPosY) >= gestureMaxY) {
            windowPosY = destination;
            output = 0;
            gestureCase = GESTURE_NOTHING;
        }

        break;

    case GESTURE_CLOSING_ACTION:
        static bool temp = false;
        static float mouseOffset;
        static float windowOffset;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (temp == false) {
                temp = true;
                mouseOffset = io.MousePos.y;
                windowOffset = windowPosY;
            }

            // do not snap the windowPos to the mousePos, only move it by the difference of the mouse movement
            windowPosY = windowOffset + (io.MousePos.y - mouseOffset);
        } else {
            temp = false;

            if ((appSizeY - windowPosY) < gestureThresholdY)
                gestureCase = GESTURE_CLOSE;

            if ((appSizeY - windowPosY) > gestureThresholdY)
                gestureCase = GESTURE_MOVE_TO_HIGHEST;

            if ((appSizeY - windowPosY) > gestureMaxY)
                gestureCase = GESTURE_TOO_HIGH;

            valueTransition.start();
            windowPosYOnLetoff = windowPosY;
        }

        break;
    }

    ImGui::SetNextWindowSize(windowSize);
    ImGui::SetNextWindowPos(ImVec2(appSizeX/2.0 - 300.0/2.0, windowPosY));
}

void ImGuiGesture::end() {
    if (!closeWindow) {
        // std::print("\n{}\n", io.MousePos.y);
        // std::print("{}\n", appSizeY);
    }
}

void ImGuiGesture::closeGesture() {
    gestureCase = GESTURE_CLOSE;
    windowPosYOnLetoff = appSizeY - gestureMaxY;
    valueTransition.start();
}