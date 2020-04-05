//
// Created by ljn on 20-1-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_CONFIG_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_CONFIG_HPP_

#include <cmath>

namespace PathOptimizationNS {

enum CarType { ACKERMANN_STEERING = 0, SKID_STEERING = 1, };
enum SmoothingMethod { FRENET = 0, CARTESIAN = 1 };
enum OptimizationMethod {K = 0, KP = 1, KPC = 2};

class Config {
public:
    Config() = default;
    // Once config is changed, some of them must be re-calculated.
    void update() {
        circle_radius_ = sqrt(pow(car_length_ / 8, 2) + pow(car_width_ / 2, 2)) + safety_margin_;
        d1_ = -3.0 / 8.0 * car_length_;
        d2_ = -1.0 / 8.0 * car_length_;
        d3_ = 1.0 / 8.0 * car_length_;
        d4_ = 3.0 / 8.0 * car_length_;
    };
    // Car param:
    CarType car_type_{ACKERMANN_STEERING};
    double car_width_{2.0};
    double car_length_{4.9};
    double safety_margin_{0.0};
    double circle_radius_{sqrt(pow(car_length_ / 8, 2) + pow(car_width_ / 2, 2)) + safety_margin_};
    double wheel_base_{2.85};
    double rear_axle_to_center_distance_{1.45}; // Distance from rear axle center to the center of the vehicle.
    double d1_{-3.0 / 8.0 * car_length_}, d2_{-1.0 / 8.0 * car_length_}, d3_{1.0 / 8.0 * car_length_}, d4_{3.0 / 8.0 * car_length_}; // Distance from vehicle center to the covering circles, from rear to front.
    double max_steer_angle_{30 * M_PI / 180};
    double mu_{0.4};
    double max_curvature_rate_{0.1}; // TODO: verify this.

    // Smoothing phase related:
    SmoothingMethod smoothing_method_{FRENET};
    bool modify_input_points_{true}; // Use A* search to generate new input points. Turn this on if the input points are of low quality.
    double a_star_lateral_range_{10};
    double a_star_longitudinal_interval_{1.5},
        a_star_lateral_interval_{0.6}; // A* search interval. Affects quality and efficiency.
    double frenet_curvature_w_{1500}, frenet_curvature_rate_w_{200}, frenet_deviation_w_{4}; // Frenet method weights.
    double cartesian_curvature_w_{10}, cartesian_deviation_w_{0.001}; // Cartesian method weights.

    // Optimization phase related:
    OptimizationMethod optimization_method_{KP};
    double opt_curvature_w_{10}, opt_curvature_rate_w_{200}, opt_deviation_w_{0}, opt_bound_slack_w_{3};
    bool constraint_end_heading_{true};
    bool exact_end_position_{false};
    double expected_safety_margin_{1.3}; // This is a soft constriant.

    // Output option
    bool raw_result_{true};
    double output_interval_{0.3};
    bool info_output_{true};
    bool check_collision_{true};
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_CONFIG_HPP_
