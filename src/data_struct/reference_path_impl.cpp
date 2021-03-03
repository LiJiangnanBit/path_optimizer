//
// Created by ljn on 20-3-23.
//
#include <cfloat>
#include <glog/logging.h>
#include "path_optimizer/data_struct/reference_path_impl.hpp"
#include <path_optimizer/tools/Map.hpp>
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/tools/spline.h"
#include "path_optimizer/data_struct/data_struct.hpp"
#include "path_optimizer/config/planning_flags.hpp"

namespace PathOptimizationNS {

ReferencePathImpl::ReferencePathImpl() :
    x_s_(new tk::spline),
    y_s_(new tk::spline),
    original_x_s_(new tk::spline),
    original_y_s_(new tk::spline) {}

ReferencePathImpl::~ReferencePathImpl() {
    delete x_s_;
    delete y_s_;
    delete original_x_s_;
    delete original_y_s_;
}

const tk::spline &ReferencePathImpl::getXS() const {
    return *x_s_;
}

const tk::spline &ReferencePathImpl::getYS() const {
    return *y_s_;
}

void ReferencePathImpl::setSpline(const tk::spline &x_s,
                                  const tk::spline &y_s,
                                  double max_s) {
    *x_s_ = x_s;
    *y_s_ = y_s;
    max_s_ = max_s;
    use_spline_ = true;
}

void ReferencePathImpl::setOriginalSpline(const PathOptimizationNS::tk::spline &x_s,
                                          const PathOptimizationNS::tk::spline &y_s,
                                          double max_s) {
    *original_x_s_ = x_s;
    *original_y_s_ = y_s;
    original_max_s_ = max_s;
    is_original_spline_set = true;
}

const tk::spline &ReferencePathImpl::getOriginalXS() const {
    return *original_x_s_;
}

const tk::spline &ReferencePathImpl::getOriginalYS() const {
    return *original_y_s_;
}

void ReferencePathImpl::setReference(const std::vector<State> &reference) {
    DLOG(INFO) << "left reference version";
    reference_states_ = reference;
    use_spline_ = false;
}

void ReferencePathImpl::setReference(const std::vector<PathOptimizationNS::State> &&reference) {
    DLOG(INFO) << "right reference version";
    reference_states_ = reference;
    use_spline_ = false;
}

void ReferencePathImpl::clear() {
    max_s_ = 0;
    reference_states_.clear();
    bounds_.clear();
    max_k_list_.clear();
    max_kp_list_.clear();
}

std::size_t ReferencePathImpl::getSize() const {
    return reference_states_.size();
}

bool ReferencePathImpl::trimStates() {
    if (bounds_.empty() || reference_states_.empty() || bounds_.size() >= reference_states_.size())
        return false;
    reference_states_.resize(bounds_.size());
    return true;
}

double ReferencePathImpl::getLength() const {
    return max_s_;
}

void ReferencePathImpl::setLength(double s) {
    max_s_ = s;
}

const std::vector<State> &ReferencePathImpl::getReferenceStates() const {
    return reference_states_;
}

const std::vector<CoveringCircleBounds> &ReferencePathImpl::getBounds() const {
    return bounds_;
}

const std::vector<double> &ReferencePathImpl::getMaxKList() const {
    return max_k_list_;
}

const std::vector<double> &ReferencePathImpl::getMaxKpList() const {
    return max_kp_list_;
}

std::vector<std::tuple<State, double, double>> ReferencePathImpl::display_abnormal_bounds() const {
    return display_set_;
}

State ReferencePathImpl::getApproxState(const State &original_state, const State &actual_state, double len) const {
    // Point on refrence.
    double x = (*x_s_)(original_state.s + len);
    double y = (*y_s_)(original_state.s + len);
    //
    State v1, v2;
    v1.x = actual_state.x - original_state.x;
    v1.y = actual_state.y - original_state.y;
    v2.x = x - original_state.x;
    v2.y = y - original_state.y;
    double proj = (v1.x*v2.x + v1.y*v2.y) / std::max(0.001, sqrt(pow(v1.x,2) + pow(v1.y,2)));
    double move_dis = fabs(len) - proj;
    // Move.
    State ret;
    int sign = len >= 0 ? 1 : -1;
    ret.x = x + sign * move_dis * cos(original_state.z);
    ret.y = y + sign * move_dis * sin(original_state.z);
    ret.z = original_state.z;
    return ret;
}

void ReferencePathImpl::updateBoundsImproved(const PathOptimizationNS::Map &map) {
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    for (const auto &state : reference_states_) {
        // Circle centers.
        State
            c0(state.x + FLAGS_d1 * cos(state.z),
               state.y + FLAGS_d1 * sin(state.z),
               state.z),
            c1(state.x + FLAGS_d2 * cos(state.z),
               state.y + FLAGS_d2 * sin(state.z),
               state.z),
            c2(state.x + FLAGS_d3 * cos(state.z),
               state.y + FLAGS_d3 * sin(state.z),
               state.z),
            c3(state.x + FLAGS_d4 * cos(state.z),
               state.y + FLAGS_d4 * sin(state.z),
               state.z);
        auto c00 = getApproxState(state, c0, FLAGS_d1);
        auto c11 = getApproxState(state, c1, FLAGS_d2);
        auto c22 = getApproxState(state, c2, FLAGS_d3);
        auto c33 = getApproxState(state, c3, FLAGS_d4);
        // Calculate boundaries.
        auto clearance_0 = getClearanceWithDirectionStrict(c00, map);
        auto offset_0 = global2Local(c0, c00).y;
        clearance_0[0] += offset_0; clearance_0[1] += offset_0;

        auto clearance_1 = getClearanceWithDirectionStrict(c11, map);
        auto offset_1 = global2Local(c1, c11).y;
        clearance_1[0] += offset_1; clearance_1[1] += offset_1;

        auto clearance_2 = getClearanceWithDirectionStrict(c22, map);
        auto offset_2 = global2Local(c2, c22).y;
        clearance_2[0] += offset_2; clearance_2[1] += offset_2;

        auto clearance_3 = getClearanceWithDirectionStrict(c33, map);
        auto offset_3 = global2Local(c3, c33).y;
        clearance_3[0] += offset_3; clearance_3[1] += offset_3;

        if (isEqual(clearance_0[0], clearance_0[1]) ||
            isEqual(clearance_1[0], clearance_1[1]) ||
            isEqual(clearance_2[0], clearance_2[1]) ||
            isEqual(clearance_3[0], clearance_3[1])) {
            LOG(INFO) << "Path is blocked at s: " << state.s;
            break;
        }
        CoveringCircleBounds covering_circle_bounds;
        covering_circle_bounds.c0 = clearance_0;
        covering_circle_bounds.c1 = clearance_1;
        covering_circle_bounds.c2 = clearance_2;
        covering_circle_bounds.c3 = clearance_3;
        bounds_.emplace_back(covering_circle_bounds);
    }
    if (reference_states_.size() != bounds_.size()) {
        reference_states_.resize(bounds_.size());
    }
}

void ReferencePathImpl::updateLimits() {
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateLimits() fail!";
        return;
    }
    if (FLAGS_optimization_method != "KPC") {
        // curvature and curvature rate can only be limited in KPC method.
        return;
    }
    max_k_list_.clear();
    max_kp_list_.clear();
    if (use_spline_) {
        LOG(ERROR) << "Reference states must be given directly!";
        // If reference_states_ are built from spline, then no speed and acc info can be used.
        for (size_t i = 0; i != reference_states_.size(); ++i) {
            max_k_list_.emplace_back(tan(FLAGS_max_steering_angle) / FLAGS_wheel_base);
            max_kp_list_.emplace_back(DBL_MAX);
        }
        return;
    }
    for (size_t i = 0; i != reference_states_.size(); ++i) {
        // Friction circle limit.
        double ref_v = reference_states_.at(i).v;
        double ref_ax = reference_states_.at(i).a;
        double ay_allowed = sqrt(pow(FLAGS_mu * 9.8, 2) - pow(ref_ax, 2));
        if (ref_v > 0.0001) max_k_list_.emplace_back(ay_allowed / pow(ref_v, 2));
        else max_k_list_.emplace_back(DBL_MAX);
        // Control rate limit.
        if (ref_v > 0.0001) max_kp_list_.emplace_back(FLAGS_max_curvature_rate / ref_v);
        else max_kp_list_.emplace_back(DBL_MAX);
    }
    LOG(INFO) << "K and KP constraints are updated according to v and a.";
}

void ReferencePathImpl::updateBounds(const Map &map) {
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    for (const auto &state : reference_states_) {
        // Circle centers.
        State
            c0(state.x + FLAGS_d1 * cos(state.z),
               state.y + FLAGS_d1 * sin(state.z),
               state.z),
            c1(state.x + FLAGS_d2 * cos(state.z),
               state.y + FLAGS_d2 * sin(state.z),
               state.z),
            c2(state.x + FLAGS_d3 * cos(state.z),
               state.y + FLAGS_d3 * sin(state.z),
               state.z),
            c3(state.x + FLAGS_d4 * cos(state.z),
               state.y + FLAGS_d4 * sin(state.z),
               state.z);
        // Calculate boundaries.
        auto clearance_0 = getClearanceWithDirectionStrict(c0, map);
        auto clearance_1 = getClearanceWithDirectionStrict(c1, map);
        auto clearance_2 = getClearanceWithDirectionStrict(c2, map);
        auto clearance_3 = getClearanceWithDirectionStrict(c3, map);
        if (clearance_0[0] == clearance_0[1] ||
            clearance_1[0] == clearance_1[1] ||
            clearance_2[0] == clearance_2[1] ||
            clearance_3[0] == clearance_3[1]) {
            LOG(INFO) << "Path is blocked at s: " << state.s;
            return;
        }
        CoveringCircleBounds covering_circle_bounds;
        covering_circle_bounds.c0 = clearance_0;
        covering_circle_bounds.c1 = clearance_1;
        covering_circle_bounds.c2 = clearance_2;
        covering_circle_bounds.c3 = clearance_3;
        bounds_.emplace_back(covering_circle_bounds);
    }
    if (reference_states_.size() != bounds_.size()) {
        reference_states_.resize(bounds_.size());
    }
    LOG(INFO) << "Boundary updated.";
}

std::vector<double> ReferencePathImpl::getClearanceWithDirectionStrict(const PathOptimizationNS::State &state,
                                                                       const PathOptimizationNS::Map &map) {
    // TODO: too much repeated code!
    double left_bound = 0;
    double right_bound = 0;
    double delta_s = 0.5;
    double left_angle = constraintAngle(state.z + M_PI_2);
    double right_angle = constraintAngle(state.z - M_PI_2);

    auto n = static_cast<size_t >(5.0 / delta_s);
    // Check if the original position is collision free.
    grid_map::Position original_position(state.x, state.y);
    auto original_clearance = map.getObstacleDistance(original_position);
    if (original_clearance > FLAGS_circle_radius) {
        // Normal case:
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance < FLAGS_circle_radius) {
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
            if (clearance < FLAGS_circle_radius) {
                break;
            }
        }
        right_bound = -(right_s - delta_s);
        left_bound = left_s - delta_s;
    } else if (is_original_spline_set && use_spline_ && !FLAGS_enable_simple_boundary_decision) {
        DLOG(INFO) << "Using relative position to determine the direction to expand.";
        // Use position to determine the direction.
        auto closest_point{findClosestPoint(*original_x_s_,
                                            *original_y_s_,
                                            state.x,
                                            state.y,
                                            original_max_s_)};
        auto local_view{global2Local(state, closest_point)};
        DLOG(INFO) << "closest point: " << closest_point.x << ", " << closest_point.y << "\n"
                   << "state: " << state.x << ", " << state.y;
        if (local_view.y < 0) {
            DLOG(INFO) << "Choose right.";
            // Expand to the right:
            double right_s = 0;
            for (int j = 0; j != n; ++j) {
                right_s += delta_s;
                double x = state.x + right_s * cos(right_angle);
                double y = state.y + right_s * sin(right_angle);
                grid_map::Position new_position(x, y);
                double clearance = map.getObstacleDistance(new_position);
                if (clearance > FLAGS_circle_radius) {
                    break;
                }
            }
            left_bound = -right_s;
            for (int j = 0; j != n; ++j) {
                right_s += delta_s;
                double x = state.x + right_s * cos(right_angle);
                double y = state.y + right_s * sin(right_angle);
                grid_map::Position new_position(x, y);
                double clearance = map.getObstacleDistance(new_position);
                if (clearance < FLAGS_circle_radius) {
                    break;
                }
            }
            right_bound = -right_s + delta_s;
        } else {
            DLOG(INFO) << "Choose left.";
            // Expand to the left:
            double left_s = 0;
            for (int j = 0; j != n; ++j) {
                left_s += delta_s;
                double x = state.x + left_s * cos(left_angle);
                double y = state.y + left_s * sin(left_angle);
                grid_map::Position new_position(x, y);
                double clearance = map.getObstacleDistance(new_position);
                if (clearance > FLAGS_circle_radius) {
                    break;
                }
            }
            right_bound = left_s;
            for (int j = 0; j != n; ++j) {
                left_s += delta_s;
                double x = state.x + left_s * cos(left_angle);
                double y = state.y + left_s * sin(left_angle);
                grid_map::Position new_position(x, y);
                double clearance = map.getObstacleDistance(new_position);
                if (clearance < FLAGS_circle_radius) {
                    break;
                }
            }
            left_bound = left_s - delta_s;
        }
        DLOG(INFO) << left_bound << ", " << right_bound;
    } else {
        // Collision already; pick one side to expand:
        DLOG(INFO) << "Using simple sampling to determine the direction to expand.";
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance > FLAGS_circle_radius) {
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
            if (clearance > FLAGS_circle_radius) {
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
                if (clearance < FLAGS_circle_radius) {
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
                if (clearance < FLAGS_circle_radius) {
                    break;
                }
            }
            right_bound = -(right_s - delta_s);
        }
    }
    // Search backward more precisely.
    double smaller_ds = 0.1;
    for (int i = 1; i != static_cast<int>(delta_s / smaller_ds); ++i) {
        left_bound += smaller_ds;
        grid_map::Position position(
            state.x + left_bound * cos(left_angle),
            state.y + left_bound * sin(left_angle)
        );
        if (map.getObstacleDistance(position) < FLAGS_circle_radius) {
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
        if (map.getObstacleDistance(position) < FLAGS_circle_radius) {
            right_bound += smaller_ds;
            break;
        }
    }
    // Only one direction:
    if (left_bound * right_bound >= 0) {
        display_set_.emplace_back(std::make_tuple(state, left_bound, right_bound));
    }
    return {left_bound, right_bound};
}

bool ReferencePathImpl::buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger) {
    CHECK_LE(delta_s_smaller, delta_s_larger);
    if (!use_spline_ || max_s_ <= 0) {
        LOG(WARNING) << "Cannot build reference line from spline!";
        return false;
    }
    reference_states_.clear();
    const double large_k = 0.2;
    const double small_k = 0.08;
    double tmp_s = 0;
    while (tmp_s <= max_s_) {
        double x = (*x_s_)(tmp_s);
        double y = (*y_s_)(tmp_s);
        double h = getHeading(*x_s_, *y_s_, tmp_s);
        double k = getCurvature(*x_s_, *y_s_, tmp_s);
        reference_states_.emplace_back(x, y, h, k, tmp_s);
        // Use k to decide delta s.
        if (FLAGS_enable_dynamic_segmentation) {
            double k_share = fabs(k) > large_k ? 1 :
                             fabs(k) < small_k ? 0 : (fabs(k) - small_k) / (large_k - small_k);
            tmp_s += delta_s_larger - k_share * (delta_s_larger - delta_s_smaller);
        } else tmp_s += delta_s_larger;
    }
    use_spline_ = true;
    return true;
}

}
