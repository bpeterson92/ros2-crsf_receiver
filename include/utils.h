#ifndef CRSF_RECEIVER_UTILS_HPP
#define CRSF_RECEIVER_UTILS_HPP

#include <chrono>
#include <algorithm>


clock_t millis(std::chrono::time_point<std::chrono::high_resolution_clock> start_time);

long convert_range(long x, double in_min, double in_max, double out_min, double out_max);

#endif