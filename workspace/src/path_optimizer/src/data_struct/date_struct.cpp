//
// Created by ljn on 20-3-12.
//
#include <cfloat>
#include <path_optimizer/tools/Map.hpp>
#include <path_optimizer/config/config.hpp>
#include "path_optimizer/data_struct/data_struct.hpp"
#include "path_optimizer/tools/tools.hpp"

namespace PathOptimizationNS {

void ReferencePath::updateLimits(const Config &config) {
    if (!reference_states) {
        LOG(WARNING) << "[SolverInput] Empty reference, updateLimits fail!";
        return;
    }
    max_k_list.clear();
    max_kp_list.clear();
    for (size_t i = 0; i != reference_states->size(); ++i) {
        // Friction circle limit.
        double ref_v = reference_states->at(i).v;
        double ref_ax = reference_states->at(i).a;
        double ay_allowed = sqrt(pow(config.mu_ * 9.8, 2) - pow(ref_ax, 2));
        if (ref_v > 0.0001) max_k_list.emplace_back(ay_allowed / pow(ref_v, 2));
        else max_k_list.emplace_back(DBL_MAX);
        // Control rate limit.
        if (ref_v > 0.0001) max_kp_list.emplace_back(config.max_curvature_rate_ / ref_v);
        else max_kp_list.emplace_back(DBL_MAX);
    }
}

void ReferencePath::updateBounds(const Map &map, const Config &config) {
    if (!reference_states) {
        LOG(WARNING) << "[SolverInput] Empty reference, updateBounds fail!";
        return;
    }
    bounds.clear();
    for (const auto &state : *reference_states) {
        double center_x = state.x + config.rear_axle_to_center_distance_ * cos(state.z);
        double center_y = state.y + config.rear_axle_to_center_distance_ * sin(state.z);
        State
            c0(center_x + config.d1_ * cos(state.z), center_y + config.d1_ * sin(state.z), state.z),
            c1(center_x + config.d2_ * cos(state.z), center_y + config.d2_ * sin(state.z), state.z),
            c2(center_x + config.d3_ * cos(state.z), center_y + config.d3_ * sin(state.z), state.z),
            c3(center_x + config.d4_ * cos(state.z), center_y + config.d4_ * sin(state.z), state.z);
        auto clearance_0 = getClearanceWithDirectionStrict(c0, map, config.circle_radius_);
        auto clearance_1 = getClearanceWithDirectionStrict(c1, map, config.circle_radius_);
        auto clearance_2 = getClearanceWithDirectionStrict(c2, map, config.circle_radius_);
        auto clearance_3 = getClearanceWithDirectionStrict(c3, map, config.circle_radius_);
        if (clearance_0[0] == clearance_0[1] ||
            clearance_1[0] == clearance_1[1] ||
            clearance_2[0] == clearance_2[1] ||
            clearance_3[0] == clearance_3[1] &&
            state.s > 0.75 * max_s_) {
            printf("some states near end are not satisfying\n");
            return;
        }
        CoveringCircleBounds covering_circle_bounds;
        covering_circle_bounds.c0 = clearance_0;
        covering_circle_bounds.c1 = clearance_1;
        covering_circle_bounds.c2 = clearance_2;
        covering_circle_bounds.c3 = clearance_3;
        bounds.emplace_back(covering_circle_bounds);
    }
}

std::vector<double> ReferencePath::getClearanceWithDirectionStrict(const PathOptimizationNS::State &state,
                                                                   const PathOptimizationNS::Map &map,
                                                                   double radius) const {
    double left_bound = 0;
    double right_bound = 0;
    double delta_s = 0.5;
    double left_angle = constraintAngle(state.z + M_PI_2);
    double right_angle = constraintAngle(state.z - M_PI_2);

    auto n = static_cast<size_t >(5.0 / delta_s);
    // Check if the original position is collision free.
    grid_map::Position original_position(state.x, state.y);
    auto original_clearance = map.getObstacleDistance(original_position);
    if (original_clearance > radius) {
        // Normal case:
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance < radius) {
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
            if (clearance < radius) {
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
            if (clearance > radius) {
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
            if (clearance > radius) {
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
                if (clearance < radius) {
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
                if (clearance < radius) {
                    break;
                }
            }
            right_bound = -(right_s - delta_s);
        }
    }
    // Search backward.
    double smaller_ds = 0.1;
    for (int i = 1; i != static_cast<int>(delta_s / smaller_ds); ++i) {
        left_bound += smaller_ds;
        grid_map::Position position(
            state.x + left_bound * cos(left_angle),
            state.y + left_bound * sin(left_angle)
        );
        if (map.getObstacleDistance(position) < radius) {
            left_bound -= smaller_ds;
            break;
        }
    }
    for (int i = 1; i != static_cast<int>(delta_s / smaller_ds); ++i) {
        right_bound -= smaller_ds;
        grid_map::Position position(
            state.x + right_bound * cos(right_angle),
            state.y + right_bound * sin(right_angle)
        );
        if (map.getObstacleDistance(position) < radius) {
            right_bound += smaller_ds;
            break;
        }
    }
    return {left_bound, right_bound};
}

}