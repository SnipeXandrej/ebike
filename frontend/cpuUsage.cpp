#include "cpuUsage.hpp"

double CPUUsage::getCPUTime(bool thread_only) {
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

void CPUUsage::measureStart(bool thread_only) {
    if (canMeasure) {
        // Record start time (wall clock) and CPU time
        wall_start = std::chrono::high_resolution_clock::now();
        cpu_start = getCPUTime(thread_only);
        canMeasure = 0;
    }
}

void CPUUsage::measureEnd(bool thread_only) {
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