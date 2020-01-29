//
// Created by ljn on 20-1-26.
//

#include "tools/tools.hpp"

// Set angle to -pi ~ pi
template<typename T>
T constraintAngle(T angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
        return constraintAngle(angle);
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
        return constraintAngle(angle);
    } else {
        return angle;
    }
}

double time_s(const clock_t &begin, const clock_t &end) {
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC;
}

double time_ms(const clock_t &begin, const clock_t &end) {
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC * 1000;
}