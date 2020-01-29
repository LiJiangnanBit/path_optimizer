//
// Created by ljn on 20-1-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
#define PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <ctime>

// Set angle to -pi ~ pi
template<typename T>
T constraintAngle(T angle);

// Output time duration in seconds.
double time_s(const clock_t &begin, const clock_t &end);

// Output time duration in ms.
double time_ms(const clock_t &begin, const clock_t &end);

#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
