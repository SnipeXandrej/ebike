#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

class Timer {
public:
    void start();
    void end();
    double getTime_us();
    double getTime_ms();
    double getTime_ms_now();
    double getTime_s();

private:
    std::chrono::high_resolution_clock::time_point timeStart;
    std::chrono::high_resolution_clock::time_point timeEnd;
    std::chrono::duration<double, std::micro> timeElapsed;
    std::chrono::duration<double, std::micro> timeElapsedNow;
};

#endif