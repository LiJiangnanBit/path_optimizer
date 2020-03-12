//
// Created by ljn on 19-8-16.
//

#ifndef PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
#define PATH_OPTIMIZER__PATHOPTIMIZER_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <memory>
#include <ros/ros.h>
#include <tinyspline_ros/tinysplinecpp.h>
#include "data_struct/data_struct.hpp"
#include "tools/collosion_checker.hpp"
#include "config/config.hpp"
#include "tools/Map.hpp"

namespace PathOptimizationNS {

class SolverKAsInput;
class PathOptimizer {
public:
    PathOptimizer() = delete;
    PathOptimizer(const State &start_state,
                  const State &end_state,
                  const grid_map::GridMap &map);

    // Change config.
    template<typename T>
    bool setConfig(const std::string &config_name, const T &value);

    // Call this to get the optimized path.
    bool solve(const std::vector<State> &reference_points, std::vector<State> *final_path);
    bool solveWithoutSmoothing(const std::vector<State> &reference_points, std::vector<State> *final_path);

    // Incomplete:
    // Sample a set of candidate paths of various longitudinal distance and lateral offset.
    // Note that it might be very slow if "densify_path" is set to false.
//    bool samplePaths(const std::vector<State> &reference_points,
//                     const std::vector<double> &lon_set,
//                     const std::vector<double> &lat_set,
//                     std::vector<std::vector<State>> *final_path_set);

    // Incomplete:
    // For dynamic obstacle avoidance. Please Ignore this.
//    bool optimizeDynamic(const std::vector<State> &reference_points,
//                         const std::vector<double> &sr_list,
//                         const std::vector<std::vector<double>> &clearance_list,
//                         std::vector<double> *x_list,
//                         std::vector<double> *y_list,
//                         std::vector<double> *s_list);

    // Get config:
    const Config &getConfig() {
        return config_;
    }

    // Only for visualization purpose.
    const std::vector<State> &getRearBounds() const;
    const std::vector<State> &getCenterBounds() const;
    const std::vector<State> &getFrontBounds() const;
    const std::vector<State> &getSmoothedPath() const;
    std::vector<std::vector<double>> a_star_display_;

private:
    // TODO: abandon this function, use the config class instead.
    void setConfig();

    // Core function.
    bool optimizePath(std::vector<State> *final_path);

    // Generate a set of paths with the same longitudinal length on reference line.
//    bool sampleSingleLongitudinalPaths(double lon,
//                                       const std::vector<double> &lat_set,
//                                       std::vector<std::vector<State>> *final_path_set,
//                                       bool max_lon_flag);

    // Get bounds for each circle at each sampling point.
//    std::vector<double> getClearanceWithDirectionStrict(const State &state, double radius) const;
//    std::shared_ptr<std::vector<double>> getClearanceFor4Circles(const State &state);

    // Divide smoothed path into segments.
    bool divideSmoothedPath();

    const Map grid_map_;
    CollisionChecker collision_checker_;
    Config config_;
    ReferencePath reference_path_;
    VehicleState vehicle_state_;
    size_t N_{};
    // For dynamic obstacle avoidace. Please Ignore this.
    std::shared_ptr<SolverKAsInput> dynamic_solver_ptr;
    bool solver_dynamic_initialized{false};
    tk::spline xsr_, ysr_;
    // For visualization purpose.
    std::vector<State> rear_bounds_;
    std::vector<State> center_bounds_;
    std::vector<State> front_bounds_;
    std::vector<State> smoothed_path_;
};

template<typename T>
bool PathOptimizer::setConfig(const std::string &config_name, const T &value) {
    if (config_name == "car_width_") {
        config_.car_width_ = static_cast<double>(value);
        config_.circle_radius_ = sqrt(pow(config_.car_length_ / 8, 2) + pow(config_.car_width_ / 2, 2));
    } else if (config_name == "car_length_") {
        config_.car_length_ = static_cast<double>(value);
        config_.circle_radius_ = sqrt(pow(config_.car_length_ / 8, 2) + pow(config_.car_width_ / 2, 2));
        config_.d1_ = -3.0 / 8.0 * config_.car_length_;
        config_.d2_ = -1.0 / 8.0 * config_.car_length_;
        config_.d3_ = 1.0 / 8.0 * config_.car_length_;
        config_.d4_ = 3.0 / 8.0 * config_.car_length_;
    } else if (config_name == "wheel_base_") {
        config_.wheel_base_ = static_cast<double>(value);
    } else if (config_name == "rear_axle_to_center_distance_") {
        config_.rear_axle_to_center_distance_ = static_cast<double>(value);
    } else if (config_name == "max_steer_angle_") {
        config_.max_steer_angle_ = static_cast<double>(value);
    } else if (config_name == "modify_input_points_") {
        config_.modify_input_points_ = static_cast<bool>(value);
    } else if (config_name == "constraint_end_heading_") {
        config_.constraint_end_heading_ = static_cast<bool>(value);
    } else if (config_name == "exact_end_position_") {
        config_.exact_end_position_ = static_cast<bool>(value);
    } else if (config_name == "expected_safety_margin_") {
        config_.expected_safety_margin_ = static_cast<double>(value);
    } else if (config_name == "raw_result_") {
        config_.raw_result_ = static_cast<bool>(value);
    } else if (config_name == "output_interval_") {
        config_.output_interval_ = static_cast<double>(value);
    } else {
        LOG(WARNING) << "[PathOptimizer] No config named " << config_name << " or this config can only be changed in config file.";
        return false;
    }
    LOG(INFO) << "[PathOptimizer] Config " << config_name << " is successfully changed to " << value << "!";
    return true;
}

}

#endif //PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
