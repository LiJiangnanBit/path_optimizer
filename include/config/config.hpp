//
// Created by ljn on 20-1-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_CONFIG_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_CONFIG_HPP_
namespace PathOptimizationNS {

#define MAX_STEER_ANGLE 35 * M_PI / 180

enum CarType { ACKERMANN_STEERING = 0, SKID_STEERING = 1, };
enum SmoothingMethod { FRENET = 0, CARTESIAN = 1 };

class Config {
public:
    Config() = default;
    // Car param:
    CarType car_type_;
    double car_width_;
    double car_length_;
    double circle_radius_;
    double wheel_base_;
    double rear_axle_to_center_distance_; // Distance from rear axle center to the center of the vehicle.
    double d1_, d2_, d3_, d4_; // Distance from vehicle center to the covering circles, from rear to front.
    double max_steer_angle_;

    // Smoothing phase related:
    SmoothingMethod smoothing_method_;
    bool modify_input_points_; // Use A* search to generate new input points. Turn this on if the input points are of low quality.
    double a_star_lateral_range_;
    double a_star_longitudinal_interval_,
        a_star_lateral_interval_; // A* search interval. Affects quality and efficiency.
    double frenet_curvature_w_, frenet_curvature_rate_w_, frenet_deviation_w_; // Frenet method weights.
    double cartesian_curvature_w_, cartesian_deviation_w_; // Cartesian method weights.

    // Optimization phase related:
    double opt_curvature_w_, opt_curvature_rate_w_, opt_deviation_w_, opt_slack_w_;
    bool constraint_end_heading_;
    bool exact_end_position_;
    double expected_safety_margin_;

    // Output option
    bool raw_result_;
    double output_interval_;
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_CONFIG_HPP_
