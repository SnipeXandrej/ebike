#include <cstdint>

class LoopRateLimiter {
public:
    void setRate(float rate);

    float getLoopRate();

    void start();

    void end();

private:
    float targetFrameTime = 1000.0 / (float)UINT64_MAX;
    float currentLoopRate = 0;
    uint64_t frameStart = 0;
    uint64_t frameEnd = 0;
    uint64_t frameTime = 0;
};