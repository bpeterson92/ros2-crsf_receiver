#include "utils.h"


clock_t millis(std::chrono::time_point<std::chrono::high_resolution_clock> start_time)
{
    const auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> fs = end - start_time;
    std::chrono::milliseconds d = std::chrono::duration_cast<std::chrono::milliseconds>(fs);
    return d.count();
}


long convert_range(long x, double in_min, double in_max, double out_min, double out_max)
{
    long v = (x - in_min) * (out_max - out_min) / (in_max - 1 - in_min) + out_min;
    return v = std::max(std::min(v, (long)out_max), (long)out_min);
}
