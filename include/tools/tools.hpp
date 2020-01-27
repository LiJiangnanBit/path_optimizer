//
// Created by ljn on 20-1-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
#define PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>

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

//double getPointCurvature(const double &x1,
//                                        const double &y1,
//                                        const double &x2,
//                                        const double &y2,
//                                        const double &x3,
//                                        const double &y3) {
//    double_t a, b, c;
//    double_t delta_x, delta_y;
//    double_t s;
//    double_t A;
//    double_t curv;
//    double_t rotate_direction;
//
//    delta_x = x2 - x1;
//    delta_y = y2 - y1;
//    a = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));
//
//    delta_x = x3 - x2;
//    delta_y = y3 - y2;
//    b = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));
//
//    delta_x = x1 - x3;
//    delta_y = y1 - y3;
//    c = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));
//
//    s = (a + b + c) / 2.0;
//    A = sqrt(fabs(s * (s - a) * (s - b) * (s - c)));
//    curv = 4 * A / (a * b * c);
//
//    /* determine the sign, using cross product(叉乘)
//     * 2维空间中的叉乘是： A x B = |A||B|Sin(\theta)
//     * V1(x1, y1) X V2(x2, y2) = x1y2 – y1x2
//     */
//    rotate_direction = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
//    if (rotate_direction < 0) {
//        curv = -curv;
//    }
//    return curv;
//}
//
//bool getCurvature(const std::vector<double> &local_x,
//                  const std::vector<double> &local_y,
//                  std::vector<double> *pt_curvature_out,
//                  double *max_curvature_abs = nullptr,
//                  double *max_curvature_change_abs = nullptr) {
//    assert(local_x.size() == local_y.size());
//    if (local_x.size() < 3) {
//        std::cout << "can't get curvature: size is " << local_x.size() << std::endl;
//        return false;
//    }
//    unsigned long size_n = local_x.size();
//    std::vector<double> curvature = std::vector<double>(size_n);
//    for (size_t i = 1; i < size_n - 1; ++i) {
//        double x1 = local_x.at(i - 1);
//        double x2 = local_x.at(i);
//        double x3 = local_x.at(i + 1);
//        double y1 = local_y.at(i - 1);
//        double y2 = local_y.at(i);
//        double y3 = local_y.at(i + 1);
//        curvature.at(i) = getPointCurvature(x1, y1, x2, y2, x3, y3);
//    }
//    curvature.at(0) = curvature.at(1);
//    curvature.at(size_n - 1) = curvature.at(size_n - 2);
//    double final_curvature;
//    double max_curvature = 0;
//    double max_curvature_change = 0;
//    for (size_t j = 0; j < size_n; ++j) {
//        final_curvature = curvature[j];
//        pt_curvature_out->emplace_back(final_curvature);
//        if (fabs(final_curvature) > max_curvature) {
//            max_curvature = fabs(final_curvature);
//        }
//        if (j != size_n - 1) {
//            double curvature_change = fabs(curvature[j] - curvature[j + 1]);
//            if (curvature_change > max_curvature_change) {
//                max_curvature_change = curvature_change;
//            }
//        }
//    }
//    if (max_curvature_abs) *max_curvature_abs = max_curvature;
//    if (max_curvature_change_abs) *max_curvature_change_abs = max_curvature_change;
//    return true;
//}

#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
