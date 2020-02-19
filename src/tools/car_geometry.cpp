//
// Created by ljn on 20-2-13.
//

#include "tools/car_geometry.hpp"
#include "tools/tools.hpp"

namespace PathOptimizationNS {

CarGeometry::CarGeometry(double width, double back_length, double front_length) :
    width_(width),
    length_(front_length + back_length),
    front_length_(front_length),
    back_length_(back_length),
    fl_p_(front_length, width / 2.0),
    fr_p_(front_length, -width / 2.0),
    rl_p_(-back_length, width / 2.0),
    rr_p_(-back_length, -width / 2.0) {
    setCircles();
}

void CarGeometry::init(double width, double back_length, double front_length) {
    width_ = width;
    length_ = back_length + front_length;
    front_length_ = front_length;
    back_length_ = back_length;
    fl_p_.x = front_length;
    fl_p_.y = width / 2.0;
    fr_p_.x = front_length;
    fr_p_.y = -width / 2.0;
    rl_p_.x = -back_length;
    rl_p_.y = width / 2.0;
    rr_p_.x = -back_length;
    rr_p_.y = -width / 2.0;
    setCircles();
}

void CarGeometry::setCircles() {
    bounding_c_.x = (front_length_ - back_length_) / 2.0;
    bounding_c_.y = 0;
    bounding_c_.r = sqrt(pow(length_ / 2, 2) + pow(width_ / 2, 2));
    double small_circle_shift = width_ / 4.0;
    double small_circle_radius = sqrt(2 * pow(small_circle_shift, 2));
    double large_circle_radius = sqrt(pow(width_, 2) + pow((length_ - width_) / 2.0, 2)) / 2;
    // rr
    circles_.emplace_back(rr_p_.x + small_circle_shift, rr_p_.y + small_circle_shift, small_circle_radius);
    // rl
    circles_.emplace_back(rl_p_.x + small_circle_shift, rl_p_.y - small_circle_shift, small_circle_radius);
    // fr
    circles_.emplace_back(fr_p_.x - small_circle_shift, fr_p_.y + small_circle_shift, small_circle_radius);
    // fl
    circles_.emplace_back(fl_p_.x - small_circle_shift, fl_p_.y - small_circle_shift, small_circle_radius);
    // fm
    circles_.emplace_back(bounding_c_.x + (length_ - width_) / 4, 0, large_circle_radius);
    // rm
    circles_.emplace_back(bounding_c_.x - (length_ - width_) / 4, 0, large_circle_radius);
}

std::vector<Circle> CarGeometry::getCircles(const PathOptimizationNS::State &pos) const {
    std::vector<Circle> result;
    for (const auto &circle : circles_) {
        State state(circle.x, circle.y);
        auto global_state = local2Global(pos, state);
        result.emplace_back(global_state.x, global_state.y, circle.r);
    }
    return result;
}

Circle CarGeometry::getBoundingCircle(const State &pos) const {
    State state(bounding_c_.x, bounding_c_.y);
    auto global_state = local2Global(pos, state);
    return {global_state.x, global_state.y, bounding_c_.r};
}

}