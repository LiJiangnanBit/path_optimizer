//
// Created by ljn on 20-1-6.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <tinyspline_ros/tinysplinecpp.h>
#include <tinyspline_ros/tinyspline.h>
struct TPoint {
    double x;
    double y;
};
int main() {
    size_t control_points_num = 10;
    tinyspline::BSpline b_spline(control_points_num, 2, 5);
    std::vector<tinyspline::real> ctrlp = std::vector<tinyspline::real>{
        10.5, 29, 22, 32.5, 33, 34, 41, 38.75, 45.5, 42.5, 48, 45.5, 55, 48, 61, 49, 66, 50, 71, 50.5};
    b_spline.setControlPoints(ctrlp);
    double interval = 0.1;
    std::vector<TPoint> result;
    for (int i = 0; i != static_cast<int>(1 / interval); ++i) {
        auto tmp_p = b_spline.eval(i * interval).result();
        TPoint p = {tmp_p[0], tmp_p[1]};
        if (i > 0) {
            std::cout << sqrt(pow(p.x - result.back().x, 2) + pow(p.y - result.back().y, 2)) << std::endl;
        }
        result.emplace_back(p);
    }
}
