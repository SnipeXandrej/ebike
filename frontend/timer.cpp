#include "timer.hpp"

void Timer::start() {
    timeStart = std::chrono::high_resolution_clock::now();
}

void Timer::end() {
    timeEnd = std::chrono::high_resolution_clock::now();
    timeElapsed = timeEnd - timeStart;
}

double Timer::getTime_us() {
    return timeElapsed.count();
}

double Timer::getTime_ms() {
    return timeElapsed.count() / 1000.0;
}

double Timer::getTime_ms_now() {
    timeElapsedNow = std::chrono::high_resolution_clock::now() - timeStart;
    return timeElapsedNow.count() / 1000.0;
}

double Timer::getTime_s() {
    return timeElapsed.count() / 1000000.0;
}