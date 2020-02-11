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
#include "tools/spline.h"
#include "data_struct/data_struct.hpp"

namespace PathOptimizationNS {

// Output value.
#define  varName(x) #x
#define  printExp(exp) std::cout << #exp << " is:\t\t" <<(exp) << std::endl

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

// Output time duration in seconds.
double time_s(const clock_t &begin, const clock_t &end);

// Output time duration in ms.
double time_ms(const clock_t &begin, const clock_t &end);

// Return true if a == b.
bool isEqual(double a, double b);

// Calculate heading for spline.
double getHeading(const tk::spline &xs, const tk::spline &ys, double s);

// Calculate curvature for spline.
double getCurvature(const tk::spline &xs, const tk::spline &ys, double tmp_s);

// Calculate distance between two points.
double distance(const State &p1, const State &p2);
}

#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
