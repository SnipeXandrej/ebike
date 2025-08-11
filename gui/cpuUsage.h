#include <sys/resource.h>
#include <chrono>

class CPUUsage {
private:
    std::chrono::high_resolution_clock::time_point wall_start;
    double cpu_start;
    std::chrono::high_resolution_clock::time_point wall_end;
    double cpu_end;
    std::chrono::duration<double> wall_elapsed;

    float addedUp;
    bool canMeasure = 1;

    // double getCPUTime() {
    //     struct rusage usage;
    //     getrusage(RUSAGE_SELF, &usage);
    //     double user = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
    //     double sys = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    //     return user + sys;
    // }

    // Get CPU time used by the calling thread
    double getCPUTime(bool thread_only) {
        if (thread_only) {
            struct timespec ts;
            if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) {
                perror("clock_gettime");
                return 0;
            }
            return ts.tv_sec + ts.tv_nsec / 1e9;
        } else {
            struct rusage usage;
            getrusage(RUSAGE_SELF, &usage);
            double user = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
            double sys = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
            return user + sys;
        }
    }

public:
    double cpu_percent;

    void measureStart(bool thread_only) {
        if (canMeasure) {
            // Record start time (wall clock) and CPU time
            wall_start = std::chrono::high_resolution_clock::now();
            cpu_start = getCPUTime(thread_only);
            canMeasure = 0;
        }
    }

    void measureEnd(bool thread_only) {
        // Record end time

        wall_end = std::chrono::high_resolution_clock::now();
        cpu_end = getCPUTime(thread_only);

        wall_elapsed = wall_end - wall_start;
        double cpu_elapsed = cpu_end - cpu_start;

        if (std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count() > 500) {
            // CPU usage percentage (can be >100% on multi-core systems)
            cpu_percent = ((cpu_elapsed / wall_elapsed.count()) * 100.0);
            canMeasure = 1;
        }
    }
};