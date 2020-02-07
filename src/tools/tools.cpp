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

bool isEqual(double a, double b) {
    return fabs(a - b) < 0.000001;
}

double getHeading(const tk::spline &xs, const tk::spline &ys, double s) {
    double x_d1 = xs.deriv(1, s);
    double y_d1 = ys.deriv(1, s);
    return atan2(y_d1, x_d1);
}