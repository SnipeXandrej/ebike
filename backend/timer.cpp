#include "timer.hpp"

void Timer::start() {
    timeElapsed = 0.0;
}

void Timer::addElapsedTime(double timeMs) {
    timeElapsed += timeMs;
}

double Timer::getTime_ms_now() {
    return timeElapsed;
}