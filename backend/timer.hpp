#ifndef TIMER_HPP
#define TIMER_HPP

class Timer {
public:
    void start();
    void addElapsedTime(double timeMs);
    double getTime_ms_now();

private:
    double timeElapsed = 0.0;
};

#endif