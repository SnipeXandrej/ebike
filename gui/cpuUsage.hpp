#include <sys/resource.h>
#include <chrono>

class CPUUsage {
private:
    std::chrono::high_resolution_clock::time_point wall_start;
    std::chrono::high_resolution_clock::time_point wall_end;
    std::chrono::duration<double> wall_elapsed;
    double cpu_start;
    double cpu_end;

    float addedUp;
    bool canMeasure = 1;

    // Get CPU time used by the calling thread
    double getCPUTime(bool thread_only);

public:
    double cpu_percent;

    void measureStart(bool thread_only);
    void measureEnd(bool thread_only);
};