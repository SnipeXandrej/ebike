#include "loopRateLimiter.hpp"
#include <chrono>
#include <thread>

void LoopRateLimiter::setRate(float rate) {
    targetFrameTime = 1000.0 / rate;
}

float LoopRateLimiter::getLoopRate() {
    return currentLoopRate;
}

void LoopRateLimiter::start() {
    frameStart = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

void LoopRateLimiter::end() {
    frameEnd = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    frameTime = frameEnd - frameStart;
    if (frameTime < targetFrameTime) {
        std::this_thread::sleep_for(std::chrono::milliseconds((uint64_t)(targetFrameTime - frameTime)));
    }

    frameEnd = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    frameTime = frameEnd - frameStart;
    currentLoopRate = 1000.0 / (float)frameTime;
}