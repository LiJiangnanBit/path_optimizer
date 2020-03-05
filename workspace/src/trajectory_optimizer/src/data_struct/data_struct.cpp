//
// Created by ljn on 20-2-21.
//
#include <glog/logging.h>
#include <path_optimizer/tools/tools.hpp>
#include "trajectory_optimizer/data_struct/data_struct.hpp"

using PathOptimizationNS::constraintAngle;
using PathOptimizationNS::getHeading;

namespace TrajOptNS {

CarType TrajOptConfig::car_type_ = PathOptimizationNS::ACKERMANN_STEERING;
double TrajOptConfig::car_width_ = 2.0;
double TrajOptConfig::car_length_ = 4.9;
double TrajOptConfig::circle_radius_ = sqrt(pow(TrajOptConfig::car_length_ / 8, 2) + pow(TrajOptConfig::car_width_ / 2, 2));
double TrajOptConfig::wheel_base_ = 2.85;
double TrajOptConfig::rear_axle_to_center_distance_ = 1.45;
double TrajOptConfig::d1_ = -3.0 / 8.0 * TrajOptConfig::car_length_;
double TrajOptConfig::d2_ = -1.0 / 8.0 * TrajOptConfig::car_length_;
double TrajOptConfig::d3_ = 1.0 / 8.0 * TrajOptConfig::car_length_;
double TrajOptConfig::d4_ = 3.0 / 8.0 * TrajOptConfig::car_length_;
double TrajOptConfig::max_steer_angle_ = 35 * M_PI / 180;
double TrajOptConfig::max_lon_acc_ = 2.0;
double TrajOptConfig::max_lon_dacc_ = -3.0;
double TrajOptConfig::max_lat_acc_ = 0.35 * 9.8;
double TrajOptConfig::safe_lat_acc_ = 0.12 * 9.8;
double TrajOptConfig::max_v_ = 13.0;
double TrajOptConfig::spacing_ = 0.3;
double TrajOptConfig::max_length_ = 80;
int TrajOptConfig::keep_control_steps_ = 4;
double TrajOptConfig::weight_ey = 1;
double TrajOptConfig::weight_v = 1;
double TrajOptConfig::weight_k = 10;
double TrajOptConfig::weight_kp = 200;
double TrajOptConfig::weight_vp = 60;
double TrajOptConfig::weight_vpp = 100;
double TrajOptConfig::weight_lat_acc_slack_ = 80;
double TrajOptConfig::weight_collision_slack_ = 25;
double TrajOptConfig::safety_margin_ = 1.2;
TrajOptConfig::SolverType TrajOptConfig::solver_type_ = TrajOptConfig::OSQP_KPVP;

void SolverInput::updateLateralBounds(const Map &map) {
    if (!reference_trajectory) {
        LOG(WARNING) << "[SolverInput] Empty reference, updateBounds fail!";
        return;
    }
    bounds.clear();
    for (const auto &state : reference_trajectory->state_list) {
        double tmp_s = state.s;
        double ref_x = reference_trajectory->x_s(tmp_s);
        double ref_y = reference_trajectory->y_s(tmp_s);
        double heading = getHeading(reference_trajectory->x_s, reference_trajectory->y_s, tmp_s);
        double center_x = ref_x + TrajOptConfig::rear_axle_to_center_distance_ * cos(heading);
        double center_y = ref_y + TrajOptConfig::rear_axle_to_center_distance_ * sin(heading);
        State
            c0(center_x + TrajOptConfig::d1_ * cos(heading), center_y + TrajOptConfig::d1_ * sin(heading), heading),
            c1(center_x + TrajOptConfig::d2_ * cos(heading), center_y + TrajOptConfig::d2_ * sin(heading), heading),
            c2(center_x + TrajOptConfig::d3_ * cos(heading), center_y + TrajOptConfig::d3_ * sin(heading), heading),
            c3(center_x + TrajOptConfig::d4_ * cos(heading), center_y + TrajOptConfig::d4_ * sin(heading), heading);
        auto clearance_0 = getClearanceWithDirectionStrict(c0, map);
        auto clearance_1 = getClearanceWithDirectionStrict(c1, map);
        auto clearance_2 = getClearanceWithDirectionStrict(c2, map);
        auto clearance_3 = getClearanceWithDirectionStrict(c3, map);
        CoveringCircleBounds covering_circle_bounds;
        covering_circle_bounds.c0 = clearance_0;
        covering_circle_bounds.c1 = clearance_1;
        covering_circle_bounds.c2 = clearance_2;
        covering_circle_bounds.c3 = clearance_3;
        bounds.emplace_back(covering_circle_bounds);
    }
}

std::vector<double> SolverInput::getClearanceWithDirectionStrict(const State &state, const Map &map) const {
    double left_bound = 0;
    double right_bound = 0;
    double delta_s = 0.2;
    double left_angle = constraintAngle(state.z + M_PI_2);
    double right_angle = constraintAngle(state.z - M_PI_2);
    auto n = static_cast<size_t >(5.0 / delta_s);
    // Check if the original position is collision free.
    grid_map::Position original_position(state.x, state.y);
    auto original_clearance = map.getObstacleDistance(original_position);
    if (original_clearance > TrajOptConfig::circle_radius_) {
        // Normal case:
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance < TrajOptConfig::circle_radius_) {
                break;
            }
        }
        double left_s = 0;
        for (size_t j = 0; j != n; ++j) {
            left_s += delta_s;
            double x = state.x + left_s * cos(left_angle);
            double y = state.y + left_s * sin(left_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance < TrajOptConfig::circle_radius_) {
                break;
            }
        }
        right_bound = -(right_s - delta_s);
        left_bound = left_s - delta_s;
    } else {
        // Collision already; pick one side to expand:
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance > TrajOptConfig::circle_radius_) {
                break;
            }
        }
        double left_s = 0;
        for (size_t j = 0; j != n; ++j) {
            left_s += delta_s;
            double x = state.x + left_s * cos(left_angle);
            double y = state.y + left_s * sin(left_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance > TrajOptConfig::circle_radius_) {
                break;
            }
        }
        // Compare
        if (left_s < right_s) {
            // Pick left side:
            right_bound = left_s;
            for (size_t j = 0; j != n; ++j) {
                left_s += delta_s;
                double x = state.x + left_s * cos(left_angle);
                double y = state.y + left_s * sin(left_angle);
                grid_map::Position new_position(x, y);
                double clearance = map.getObstacleDistance(new_position);
                if (clearance < TrajOptConfig::circle_radius_) {
                    break;
                }
            }
            left_bound = left_s - delta_s;
        } else {
            // Pick right side:
            left_bound = -right_s;
            for (size_t j = 0; j != n; ++j) {
                right_s += delta_s;
                double x = state.x + right_s * cos(right_angle);
                double y = state.y + right_s * sin(right_angle);
                grid_map::Position new_position(x, y);
                double clearance = map.getObstacleDistance(new_position);
                if (clearance < TrajOptConfig::circle_radius_) {
                    break;
                }
            }
            right_bound = -(right_s - delta_s);
        }
    }
    return {left_bound, right_bound};
}
}
