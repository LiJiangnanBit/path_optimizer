//
// Created by ljn on 20-2-13.
//

#ifndef PATH_OPTIMIZER_INCLUDE_TOOLS_CAR_GEOMETRY_HPP_
#define PATH_OPTIMIZER_INCLUDE_TOOLS_CAR_GEOMETRY_HPP_

#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_CAR_GEOMETRY_HPP_

#include <iostream>
#include <vector>
#include "data_struct/data_struct.hpp"

namespace PathOptimizationNS {

class CarGeometry {
public:
    CarGeometry() = default;
    CarGeometry(double width, double back_length, double front_length);
    void init(double width, double back_length, double front_length);
    std::vector<Circle> getCircles(const State &pos) const;
    Circle getBoundingCircle(const State &pos) const;

private:
    void setCircles();
    double width_{}, length_{}, front_length_{}, back_length_{};
    // Four corners:
    // f: front, r: rear
    // l: left, r: right
    State fl_p_, fr_p_, rl_p_, rr_p_;
    std::vector<Circle> circles_;
    Circle bounding_c_;
};
}