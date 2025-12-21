#include "imguiGestures.hpp"
#include "other.hpp"
#include <print>
#include "../valueTransition.hpp"
#include "../cubicBezier.hpp"

ValueTransition valueTransition;

// cubic bezier curve
float c_x1 = 0.16f;
float c_y1 = 0.08f;
float c_x2 = 0.0f;
float c_y2 = 0.88f;

bool closeWindow = false;
bool continueGesture = false;
float animationMoveOffset;
float appSizeY = 0;
float appSizeX = 0;
float windowPosY = 0;
float windowPosYOnLetoff = 0;

ImVec2 windowSize = ImVec2(600, 350);
float gestureMaxY = 400;
float gestureThresholdY = 150;
float gestureMouseMovementThreshold = 30;
float headerOffset = 15;

enum {
    GESTURE_NOTHING = 0,
    GESTURE_START = 1,
    GESTURE_HIDE = 2,
    GESTURE_TOO_HIGH = 3,
    GESTURE_CLOSE = 4,
    GESTURE_MOVE_TO_HIGHEST = 5,
    GESTURE_CLOSING_ACTION = 6,
    GESTURE_PRESTART = 7,
};

int gestureCase = GESTURE_NOTHING;

void f_gestureClose(float _windowPosYOnLetoff) {
    gestureCase = GESTURE_CLOSE;
    windowPosYOnLetoff = _windowPosYOnLetoff;
}

void f_gestureClose() {
    f_gestureClose(appSizeY - gestureMaxY);
}

bool wasGestureWithinStartRegion = false;
static float output;
static float output_bezier;
static float destination;


void ImGuiGesture::start() {
    appSizeY = ImGui::GetWindowSize().y;
    appSizeX = ImGui::GetWindowSize().x;

    bool isGestureWithinStartRegion = ((io.MousePos.y < appSizeY) && (io.MousePos.y > appSizeY - 55.0)) ? true : false;
    bool isHeaderWithinConstraints = (io.MousePos.y > (appSizeY - gestureMaxY)) && (io.MousePos.y < (appSizeY - gestureMaxY + 40.0))
                                  && (io.MousePos.x > (appSizeX/2.0 - (windowSize.x/2.0)) && io.MousePos.x < (appSizeX/2.0 + (windowSize.x/2.0))) ? true : false;

    // std::print("case: {}\n", gestureCase);
    switch (gestureCase) {
    case GESTURE_NOTHING:
        // hide it
        if (!continueGesture) {
            windowPosY = appSizeY;

            if ((ImGui::IsMouseDown(ImGuiMouseButton_Left) && isGestureWithinStartRegion ) || wasGestureWithinStartRegion) {
                wasGestureWithinStartRegion = true;
                static bool temp = false;
                static float initialMousePos;

                if (temp == false) {
                    temp = true;
                    initialMousePos = io.MousePos.y;
                }

                if ((initialMousePos - io.MousePos.y) > gestureMouseMovementThreshold) {
                    temp = false;
                    gestureCase = GESTURE_PRESTART;
                    valueTransition.start();
                }
            }
        }

        if (continueGesture) {
            if (isHeaderWithinConstraints) {
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    gestureCase = GESTURE_CLOSING_ACTION;
                }
            }
        }

        valueTransition.start();
        break;

    case GESTURE_PRESTART:
        output = valueTransition.getValueDifference(0.0, 100.0, 200.0);
        output_bezier = mapToCubicBezier(output, c_x1, c_y1, c_x2, c_y2) + 1.0;

        destination = io.MousePos.y - headerOffset;
        windowPosY = map_f_nochecks(output_bezier, 0, 100, appSizeY, destination);

        if (windowPosY <= destination || !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            output = 0;
            gestureCase = GESTURE_START;
            valueTransition.start();

            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                windowPosYOnLetoff = windowPosY;
            }
        }

        break;

    case GESTURE_START:
        wasGestureWithinStartRegion = false;

        if (continueGesture == false) {
            continueGesture = true;
        }

        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            windowPosY = io.MousePos.y - headerOffset;
        } else {
            valueTransition.start();
            // when the mouse goes off of the app the value is set to FLT_MAX, that is undesirable
            windowPosYOnLetoff = windowPosY > 0 ? windowPosY : 0;

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

        break;

    case GESTURE_HIDE:
        continueGesture = false;

        gestureCase = GESTURE_NOTHING;
        break;

    case GESTURE_TOO_HIGH:
        output = valueTransition.getValueDifference(0.0, 100.0, 300.0); // move down

        output_bezier = mapToCubicBezier(output, c_x1, c_y1, c_x2, c_y2) + 1.0;

        destination = appSizeY - gestureMaxY;

        windowPosY = map_f_nochecks(output_bezier, 0, 100, windowPosYOnLetoff, destination);

        if ((appSizeY - windowPosY) <= gestureMaxY) {
            windowPosY = destination;
            output = 0;
            gestureCase = GESTURE_NOTHING;
        }

        break;

    case GESTURE_CLOSE:
        output = valueTransition.getValueDifference(0.0, 100.0, 150.0); // move down

        output_bezier = mapToCubicBezier(output, c_x1, c_y1, c_x2, c_y2);

        destination = appSizeY;

        windowPosY = map_f_nochecks(output_bezier, 0, 100, windowPosYOnLetoff, destination);

        if ((int)windowPosY + 1 >= (int)destination) {
            gestureCase = GESTURE_HIDE;
            output = 0;
        }

        break;

    case GESTURE_MOVE_TO_HIGHEST:
        output = valueTransition.getValueDifference(0.0, 100.0, 300.0); // move down

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

            windowPosYOnLetoff = windowPosY;
        }

        valueTransition.start();

        break;
    }

    ImGui::SetNextWindowSize(windowSize);
    ImGui::SetNextWindowPos(ImVec2(appSizeX/2.0 - windowSize.x/2.0, windowPosY));
}

void ImGuiGesture::end() {
    if (!closeWindow) {
        // std::print("\n{}\n", io.MousePos.y);
        // std::print("{}\n", appSizeY);
    }
}

void ImGuiGesture::closeGesture() {
    f_gestureClose();
}