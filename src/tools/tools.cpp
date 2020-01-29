//
// Created by ljn on 20-1-26.
//

#include "tools/tools.hpp"

double time_s(const clock_t &begin, const clock_t &end) {
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC;
}

double time_ms(const clock_t &begin, const clock_t &end) {
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC * 1000;
}