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

double getCurvature(const tk::spline &xs, const tk::spline &ys, double tmp_s) {
    double x_d1 = xs.deriv(1, tmp_s);
    double y_d1 = ys.deriv(1, tmp_s);
    double x_d2 = xs.deriv(2, tmp_s);
    double y_d2 = ys.deriv(2, tmp_s);
    return (x_d1 * y_d2 - y_d1 * x_d2) / pow(pow(x_d1, 2) + pow(y_d1, 2), 1.5);
}