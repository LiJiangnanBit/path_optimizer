//
// Created by ljn on 20-2-9.
//
#include <glog/logging.h>
#include "path_optimizer/reference_path_smoother/reference_path_smoother.hpp"
#include "path_optimizer/tools/spline.h"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/tools/Map.hpp"
#include "path_optimizer/config/planning_flags.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/reference_path_smoother/angle_diff_smoother.hpp"
#include "path_optimizer/reference_path_smoother/tension_smoother.hpp"
#include "path_optimizer/reference_path_smoother/tension_smoother_2.hpp"
#include "OsqpEigen/OsqpEigen.h"

namespace PathOptimizationNS {

std::unique_ptr<ReferencePathSmoother> ReferencePathSmoother::create(const std::string &type,
                                                                     const std::vector<State> &input_points,
                                                                     const State &start_state,
                                                                     const Map &grid_map) {
    if (type == "ANGLE_DIFF") {
        return std::unique_ptr<ReferencePathSmoother>{new AngleDiffSmoother(input_points, start_state, grid_map)};
    } else if (type == "TENSION") {
        return std::unique_ptr<ReferencePathSmoother>{new TensionSmoother(input_points, start_state, grid_map)};
    } else if (type == "TENSION2") {
        return std::unique_ptr<ReferencePathSmoother>{new TensionSmoother2(input_points, start_state, grid_map)};
    } else {
        LOG(ERROR) << "No such smoother!";
        return nullptr;
    }
}

bool ReferencePathSmoother::solve(PathOptimizationNS::ReferencePath *reference_path) {
    // TODO: deal with short reference path.
    if (input_points_.size() < 4) {
        LOG(ERROR) << "Few reference points.";
        return false;
    }

    bSpline();

    if (!smooth(reference_path)) return false;

    if (!graphSearchDp(reference_path)) return false;

    return postSmooth(reference_path);
}

bool ReferencePathSmoother::segmentRawReference(std::vector<double> *x_list,
                                                std::vector<double> *y_list,
                                                std::vector<double> *s_list,
                                                std::vector<double> *angle_list,
                                                std::vector<double> *k_list) const {
    if (s_list_.size() != x_list_.size() || s_list_.size() != y_list_.size()) {
        LOG(ERROR) << "Raw path x y and s size not equal!";
        return false;
    }
    double max_s = s_list_.back();
    tk::spline x_spline, y_spline;
    x_spline.set_points(s_list_, x_list_);
    y_spline.set_points(s_list_, y_list_);
    // Divide the raw path.
    double delta_s = 1.0;
    s_list->emplace_back(0);
    while (s_list->back() < max_s) {
        s_list->emplace_back(s_list->back() + delta_s);
    }
    if (max_s - s_list->back() > 1) {
        s_list->emplace_back(max_s);
    }
    auto point_num = s_list->size();
    // Store reference states in vectors. They will be used later.
    for (size_t i = 0; i != point_num; ++i) {
        double length_on_ref_path = s_list->at(i);
        double dx = x_spline.deriv(1, length_on_ref_path);
        double dy = y_spline.deriv(1, length_on_ref_path);
        double ddx = x_spline.deriv(2, length_on_ref_path);
        double ddy = y_spline.deriv(2, length_on_ref_path);
        double angle = atan2(dy, dx);
        angle_list->emplace_back(angle);
        double curvature = (dx * ddy - dy * ddx) / pow(dx * dx + dy * dy, 1.5);
        k_list->emplace_back(curvature);
        x_list->emplace_back(x_spline(length_on_ref_path));
        y_list->emplace_back(y_spline(length_on_ref_path));
    }
    return true;
}

std::vector<std::vector<double>> ReferencePathSmoother::display() const {
    return std::vector<std::vector<double>>{x_list_, y_list_, s_list_};
}

double ReferencePathSmoother::getG(const PathOptimizationNS::APoint &point,
                                   const PathOptimizationNS::APoint &parent) const {
    // Obstacle cost.
    grid_map::Position position(point.x, point.y);
    double obstacle_cost = 0;
    double distance_to_obs = grid_map_.getObstacleDistance(position);
    double safety_distance = 5;
    if (distance_to_obs < safety_distance) {
        obstacle_cost = (safety_distance - distance_to_obs) / safety_distance * FLAGS_search_obstacle_cost;
    }
    // Deviation cost.
    double offset_cost = fabs(point.l) / FLAGS_search_lateral_range * FLAGS_search_deviation_cost;
    // Smoothness cost.
    return parent.g + offset_cost + obstacle_cost;
}

void ReferencePathSmoother::calculateCostAt(std::vector<std::vector<DpPoint>> &samples,
                                            int layer_index,
                                            int lateral_index) const {
    if (layer_index == 0) return;
    if (!samples[layer_index][lateral_index].is_feasible_) return;

    static const double weight_ref_offset = 1.0;
    static const double weight_obstacle = 0.5;
    static const double weight_angle_change = 16.0;
    static const double weight_ref_angle_diff = 0.5;
    static const double safe_distance = 3.0;

    auto &point = samples[layer_index][lateral_index];
    double self_cost = 0;
    if (point.dis_to_obs_ < safe_distance) self_cost += (safe_distance - point.dis_to_obs_) / safe_distance * weight_obstacle;
    self_cost += fabs(point.l_) / FLAGS_search_lateral_range * weight_ref_offset;

    auto min_cost = DBL_MAX;
    for (const auto &pre_point : samples[layer_index - 1]) {
        if (!pre_point.is_feasible_) continue;
        if (fabs(pre_point.l_ - point.l_) > (point.s_ - pre_point.s_)) continue;
        double direction = atan2(point.y_ - pre_point.y_, point.x_ - pre_point.x_);
        double edge_cost = fabs(constraintAngle(direction - pre_point.dir_)) / M_PI_2 * weight_angle_change
            + fabs(constraintAngle(direction - point.heading_)) / M_PI_2 * weight_ref_angle_diff;
        double total_cost = self_cost + edge_cost + pre_point.cost_;
        if (total_cost < min_cost) {
            min_cost = total_cost;
            point.parent_ = &pre_point;
            point.dir_ = direction;
        }
    }

    if (point.parent_) point.cost_ = min_cost;
}

bool ReferencePathSmoother::graphSearchDp(PathOptimizationNS::ReferencePath *reference) {
    auto t1 = std::clock();

    const tk::spline &x_s = reference->getXS();
    const tk::spline &y_s = reference->getYS();
    // Sampling interval.
    double tmp_s = findClosestPoint(x_s, y_s, start_state_.x, start_state_.y, reference->getLength()).s;
    layers_s_list_.clear();
    layers_bounds_.clear();
    double search_ds = reference->getLength() > 6 ? FLAGS_search_longitudial_spacing : 0.5;
    while (tmp_s < reference->getLength()) {
        layers_s_list_.emplace_back(tmp_s);
        tmp_s += search_ds;
    }
    layers_s_list_.emplace_back(reference->getLength());
    if (layers_s_list_.empty()) return false;
    target_s_ = layers_s_list_.back();

    double vehicle_s = layers_s_list_.front();
    State proj_point(x_s(vehicle_s), y_s(vehicle_s), getHeading(x_s, y_s, vehicle_s));
    auto vehicle_local = global2Local(proj_point, start_state_);
    vehicle_l_wrt_smoothed_ref_ = vehicle_local.y;
    if (fabs(vehicle_local.y) > FLAGS_search_lateral_range) {
        LOG(INFO) << "Vehicle far from ref, quit graph search.";
        return false;
    }
    int start_lateral_index =
        static_cast<int>((FLAGS_search_lateral_range + vehicle_local.y) / FLAGS_search_lateral_spacing);

    std::vector<std::vector<DpPoint>> samples;
    samples.reserve(layers_s_list_.size());
    static const double search_threshold = 1.45;
    // Sample nodes.
    for (int i = 0; i < layers_s_list_.size(); ++i) {
        samples.emplace_back(std::vector<DpPoint>());
        double cur_s = layers_s_list_[i];
        double ref_x = x_s(cur_s);
        double ref_y = y_s(cur_s);
        double ref_heading = getHeading(x_s, y_s, cur_s);
        double ref_curvature = getCurvature(x_s, y_s, cur_s);
        double ref_r = 1 / ref_curvature;
        double cur_l = -FLAGS_search_lateral_range;
        int lateral_index = 0;
        while (cur_l <= FLAGS_search_lateral_range) {
            DpPoint dp_point;
            dp_point.x_ = ref_x + cur_l * cos(ref_heading + M_PI_2);
            dp_point.y_ = ref_y + cur_l * sin(ref_heading + M_PI_2);
            dp_point.heading_ = ref_heading;
            dp_point.s_ = cur_s;
            dp_point.l_ = cur_l;
            dp_point.layer_index_ = i;
            dp_point.lateral_index_ = lateral_index;
            grid_map::Position node_pose(dp_point.x_, dp_point.y_);
            dp_point.dis_to_obs_ = grid_map_.isInside(node_pose) ? grid_map_.getObstacleDistance(node_pose) : -1;
            if ((ref_curvature < 0 && cur_l < ref_r) || (ref_curvature > 0 && cur_l > ref_r)
                || dp_point.dis_to_obs_ < search_threshold) {
                dp_point.is_feasible_ = false;
            }
            if (i == 0 && dp_point.lateral_index_ != start_lateral_index) dp_point.is_feasible_ = false;
            if (i == 0 && dp_point.lateral_index_ == start_lateral_index) {
                dp_point.is_feasible_ = true;
                dp_point.dir_ = start_state_.z;
                dp_point.cost_ = 0.0;
            }
            samples.back().emplace_back(dp_point);
            cur_l += FLAGS_search_lateral_spacing;
            ++lateral_index;
        }
        // Get rough bounds.
        auto &point_set = samples.back();
        for (int j = 0; j < point_set.size(); ++j) {
            if (j == 0 || !point_set[j - 1].is_feasible_ || !point_set[j].is_feasible_) {
                point_set[j].rough_lower_bound = point_set[j].l_;
            } else {
                point_set[j].rough_lower_bound = point_set[j - 1].rough_lower_bound;
            }
        }
        for (int j = point_set.size() - 1; j >= 0; --j) {
            if (j == point_set.size() - 1 || !point_set[j + 1].is_feasible_ || !point_set[j].is_feasible_) {
                point_set[j].rough_upper_bound = point_set[j].l_;
            } else {
                point_set[j].rough_upper_bound = point_set[j + 1].rough_upper_bound;
            }
        }
    }

    // Calculate cost.
    int max_layer_reached = 0;
    for (const auto &layer : samples) {
        bool is_layer_feasible = false;
        for (const auto &point : layer) {
            calculateCostAt(samples, point.layer_index_, point.lateral_index_);
            if (point.parent_) is_layer_feasible = true;
        }
        if (layer.front().layer_index_ != 0 && !is_layer_feasible) break;
        max_layer_reached = layer.front().layer_index_;
    }

    // Retrieve path.
    const DpPoint *ptr = nullptr;
    auto min_cost = DBL_MAX;
    for (const auto &point : samples[max_layer_reached]) {
        if (point.cost_ < min_cost) {
            ptr = &point;
            min_cost = point.cost_;
        }
    }

    while (ptr) {
        if (ptr->layer_index_ == 0) {
            layers_bounds_.emplace_back(-10, 10);
        } else {
            static const double check_s = 0.2;
            double upper_bound = check_s + ptr->rough_upper_bound;
            double lower_bound = -check_s + ptr->rough_lower_bound;
            static const double check_limit = 6.0;
            double ref_x = x_s(ptr->s_);
            double ref_y = y_s(ptr->s_);
            while (upper_bound < check_limit) {
                grid_map::Position pos;
                pos(0) = ref_x + upper_bound * cos(ptr->heading_ + M_PI_2);
                pos(1) = ref_y + upper_bound * sin(ptr->heading_ + M_PI_2);
                if (grid_map_.isInside(pos)
                    && grid_map_.getObstacleDistance(pos) > search_threshold) {
                    upper_bound += check_s;
                } else {
                    upper_bound -= check_s;
                    break;
                }
            }
            while (lower_bound > -check_limit) {
                grid_map::Position pos;
                pos(0) = ref_x + lower_bound * cos(ptr->heading_ + M_PI_2);
                pos(1) = ref_y + lower_bound * sin(ptr->heading_ + M_PI_2);
                if (grid_map_.isInside(pos)
                    && grid_map_.getObstacleDistance(pos) > search_threshold) {
                    lower_bound -= check_s;
                } else {
                    lower_bound += check_s;
                    break;
                }
            }
            layers_bounds_.emplace_back(lower_bound, upper_bound);
        }
        ptr = ptr->parent_;
    }

    std::reverse(layers_bounds_.begin(), layers_bounds_.end());
    layers_s_list_.resize(layers_bounds_.size());

    auto t2 = std::clock();
    if (FLAGS_enable_computation_time_output) {
        time_ms_out(t1, t2, "Search");
    }
    return true;

}

bool ReferencePathSmoother::graphSearch(ReferencePath *reference) {
    auto t1 = std::clock();
    tk::spline x_s = reference->getXS();
    tk::spline y_s = reference->getYS();
    // Sampling interval.
    double tmp_s = findClosestPoint(x_s, y_s, start_state_.x, start_state_.y, reference->getLength()).s;
    layers_s_list_.clear();
    layers_bounds_.clear();
    double search_ds = reference->getLength() > 6 ? FLAGS_search_longitudial_spacing : 0.5;
    while (tmp_s < reference->getLength()) {
        layers_s_list_.emplace_back(tmp_s);
        tmp_s += search_ds;
    }
    layers_s_list_.emplace_back(reference->getLength());
    target_s_ = layers_s_list_.back();

    double vehicle_s = layers_s_list_.front();
    State proj_point(x_s(vehicle_s), y_s(vehicle_s), getHeading(x_s, y_s, vehicle_s));
    auto vehicle_local = global2Local(proj_point, start_state_);
    vehicle_l_wrt_smoothed_ref_ = vehicle_local.y;

    // Sample points.
    static const double search_k = 1.2;
    APoint start_point;
    start_point.x = start_state_.x;
    start_point.y = start_state_.y;
    start_point.dir = proj_point.z;
    start_point.s = layers_s_list_.front();
    start_point.l = vehicle_local.y;
    start_point.layer = 0;
    start_point.g = 0;
    start_point.h = getH(start_point);
    sampled_points_.emplace_back(std::vector<APoint>{start_point});
    for (size_t i = 1; i != layers_s_list_.size(); ++i) {
        double sr = layers_s_list_[i];
        double xr = x_s(sr);
        double yr = y_s(sr);
        double hr = getHeading(x_s, y_s, sr);
        double rr = 1.0 / (getCurvature(x_s, y_s, sr));
        double left_range = FLAGS_search_lateral_range, right_range = -FLAGS_search_lateral_range;
        if (rr > 0) {
            // Left turn
            left_range = std::min(left_range, rr);
        } else {
            // right turn
            right_range = std::max(right_range, rr);
        }
        std::vector<APoint> point_set;
        double offset = right_range;
        int offset_idx = 0;
        while (offset <= left_range) {
            APoint point;
            point.s = sr;
            point.l = offset;
            point.x = xr + offset * cos(hr + M_PI_2);
            point.y = yr + offset * sin(hr + M_PI_2);
            point.dir = hr;
            point.layer = i;
            point.offset_idx = offset_idx;
            grid_map::Position position(point.x, point.y);
            if (grid_map_.isInside(position)
                && grid_map_.getObstacleDistance(position) > search_k * FLAGS_circle_radius) {
                point_set.emplace_back(point);
            }
            offset += FLAGS_search_lateral_spacing;
            ++offset_idx;
        }
        // Get rough bounds.
        for (int i = 0; i < point_set.size(); ++i) {
            if (i == 0 || point_set[i].offset_idx != point_set[i - 1].offset_idx + 1) {
                point_set[i].rough_lower_bound = point_set[i].l;
            } else {
                point_set[i].rough_lower_bound = point_set[i - 1].rough_lower_bound;
            }
        }
        for (int i = point_set.size() - 1; i >= 0; --i) {
            if (i == point_set.size() - 1 || point_set[i].offset_idx != point_set[i + 1].offset_idx - 1) {
                point_set[i].rough_upper_bound = point_set[i].l;
            } else {
                point_set[i].rough_upper_bound = point_set[i + 1].rough_upper_bound;
            }
        }
        sampled_points_.emplace_back(point_set);
    }

    // Push the start point into the open set.
    open_set_.push(&sampled_points_[0][0]);
    sampled_points_[0][0].is_in_open_set = true;

    // Search.
    int max_layer_reached = 0;
    while (true) {
        if (open_set_.empty()) {
            LOG(ERROR) << "Lattice search failed!";
            break;
        }
        auto tmp_point_ptr = open_set_.top();
        if (isEqual(tmp_point_ptr->s, target_s_)) {
            break;
        }
        open_set_.pop();
        max_layer_reached = std::max(max_layer_reached, tmp_point_ptr->layer);
        for (auto &child : sampled_points_[tmp_point_ptr->layer + 1]) {
            // If angle difference is too large, skip it.
            if (fabs(atan2(child.l - tmp_point_ptr->l, child.s - tmp_point_ptr->s)) > 60 * M_PI / 180) {
                continue;
            }
            // If already exsit in closet set, skip it.
            if (checkExistenceInClosedSet(child)) {
                continue;
            }
            if (child.is_in_open_set) {
                double new_g = getG(child, *tmp_point_ptr);
                if (new_g < child.g) {
                    child.g = new_g;
                    child.parent = tmp_point_ptr;
                }
            } else {
                child.g = getG(child, *tmp_point_ptr);
                child.h = getH(child);
                child.parent = tmp_point_ptr;
                open_set_.push(&child);
                child.is_in_open_set = true;
            }
        }
        closed_set_.insert(tmp_point_ptr);
    }

    // Retrieve the optimal path.
    APoint *ptr = nullptr;
    if (open_set_.empty()) {
        auto min_cost = DBL_MAX;
        for (int i = 0; i < sampled_points_[max_layer_reached].size(); ++i) {
            if (sampled_points_[max_layer_reached][i].parent == nullptr) continue;
            if (sampled_points_[max_layer_reached][i].f() < min_cost) {
                ptr = &sampled_points_[max_layer_reached][i];
                min_cost = ptr->f();
            }
        }
    } else {
        ptr = open_set_.top();
    }

    while (ptr) {
        if (ptr->layer == 0) {
            layers_bounds_.emplace_back(-10, 10);
        } else {
            static const double check_s = 0.2;
            double upper_bound = check_s + ptr->rough_upper_bound;
            double lower_bound = -check_s + ptr->rough_lower_bound;
            static const double check_limit = 6.0;
            double ref_x = x_s(ptr->s);
            double ref_y = y_s(ptr->s);
            while (upper_bound < check_limit) {
                grid_map::Position pos;
                pos(0) = ref_x + upper_bound * cos(ptr->dir + M_PI_2);
                pos(1) = ref_y + upper_bound * sin(ptr->dir + M_PI_2);
                if (grid_map_.isInside(pos)
                    && grid_map_.getObstacleDistance(pos) > 1.3 * FLAGS_circle_radius) {
                    upper_bound += check_s;
                } else {
                    upper_bound -= check_s;
                    break;
                }
            }
            while (lower_bound > -check_limit) {
                grid_map::Position pos;
                pos(0) = ref_x + lower_bound * cos(ptr->dir + M_PI_2);
                pos(1) = ref_y + lower_bound * sin(ptr->dir + M_PI_2);
                if (grid_map_.isInside(pos)
                    && grid_map_.getObstacleDistance(pos) > search_k * FLAGS_circle_radius) {
                    lower_bound -= check_s;
                } else {
                    lower_bound += check_s;
                    break;
                }
            }
            layers_bounds_.emplace_back(lower_bound, upper_bound);
        }
        ptr = ptr->parent;
    }
    std::reverse(layers_bounds_.begin(), layers_bounds_.end());
    layers_s_list_.resize(layers_bounds_.size());

    auto t2 = std::clock();
    if (FLAGS_enable_computation_time_output) {
        time_ms_out(t1, t2, "Search");
    }
    return true;
}

bool ReferencePathSmoother::checkExistenceInClosedSet(const APoint &point) const {
    return closed_set_.find(&point) != closed_set_.end();
}

void ReferencePathSmoother::bSpline() {
    // B spline smoothing.
    double length = 0;
    for (size_t i = 0; i != input_points_.size() - 1; ++i) {
        length += distance(input_points_[i], input_points_[i + 1]);
    }
    int degree = 3;
    double average_length = length / (input_points_.size() - 1);
    if (average_length > 10) degree = 3;
    else if (average_length > 5) degree = 4;
    else degree = 5;
    tinyspline::BSpline b_spline_raw(input_points_.size(), 2, degree);
    std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
    for (size_t i = 0; i != input_points_.size(); ++i) {
        ctrlp_raw[2 * (i)] = input_points_[i].x;
        ctrlp_raw[2 * (i) + 1] = input_points_[i].y;
    }
    b_spline_raw.setControlPoints(ctrlp_raw);
    double delta_t = 1.0 / length;
    double tmp_t = 0;
    while (tmp_t < 1) {
        auto result = b_spline_raw.eval(tmp_t).result();
        x_list_.emplace_back(result[0]);
        y_list_.emplace_back(result[1]);
        tmp_t += delta_t;
    }
    auto result = b_spline_raw.eval(1).result();
    x_list_.emplace_back(result[0]);
    y_list_.emplace_back(result[1]);
    s_list_.emplace_back(0);
    for (size_t i = 1; i != x_list_.size(); ++i) {
        double dis = sqrt(pow(x_list_[i] - x_list_[i - 1], 2) + pow(y_list_[i] - y_list_[i - 1], 2));
        s_list_.emplace_back(s_list_.back() + dis);
    }
}

bool ReferencePathSmoother::postSmooth(PathOptimizationNS::ReferencePath *reference_path) {
    auto point_num = layers_s_list_.size();
    if (point_num < 4) {
        LOG(WARNING) << "Ref is short: " << point_num << ", quit POST SMOOTHING.";
        return false;
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(3 * point_num);
    solver.data()->setNumberOfConstraints(3 * point_num - 2);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * point_num);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    setPostHessianMatrix(&hessian);
    setPostConstraintMatrix(&linearMatrix, &lowerBound, &upperBound);
    // Input to solver.
    if (!solver.data()->setHessianMatrix(hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver.initSolver()) return false;
    if (!solver.solve()) {
        LOG(ERROR) << "Post smoothing QP failed!";
        return false;
    }
    const auto &QPSolution = solver.getSolution();

    std::vector<double> x_list, y_list, s_list;
    const auto &ref_xs = reference_path->getXS();
    const auto &ref_ys = reference_path->getYS();
    double s = 0;
    for (int i = 0; i < point_num; ++i) {
        const double ref_s = layers_s_list_[i];
        double ref_dir = getHeading(ref_xs, ref_ys, ref_s);
        x_list.push_back(ref_xs(ref_s) + QPSolution(i) * cos(ref_dir + M_PI_2));
        y_list.push_back(ref_ys(ref_s) + QPSolution(i) * sin(ref_dir + M_PI_2));
        if (i > 0) {
            s += sqrt(pow(x_list[i] - x_list[i - 1], 2) + pow(y_list[i] - y_list[i - 1], 2));
        }
        s_list.push_back(s);
    }
    tk::spline new_x_s, new_y_s;
    new_x_s.set_points(s_list, x_list);
    new_y_s.set_points(s_list, y_list);
    reference_path->setSpline(new_x_s, new_y_s, s_list.back());

    return true;
}

void ReferencePathSmoother::setPostHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const {
    size_t size = layers_s_list_.size();
    const size_t matrix_size = 3 * size;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    // TODO: config
    const double weight_x = 1;
    const double weight_dx = 100;
    const double weight_ddx = 1000;
    for (int i = 0; i < size; ++i) {
        hessian(i, i) = weight_x;
        hessian(size + i, size + i) = weight_dx;
        hessian(2 * size + i, 2 * size + i) = weight_ddx;
    }
    *matrix_h = hessian.sparseView();
}

void ReferencePathSmoother::setPostConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                                    Eigen::VectorXd *lower_bound,
                                                    Eigen::VectorXd *upper_bound) const {
    size_t size = layers_s_list_.size();
    const size_t x_index = 0;
    const size_t dx_index = x_index + size;
    const size_t ddx_index = dx_index + size;
    const size_t cons_x_index = 0;
    const size_t cons_dx_x_index = cons_x_index + size;
    const size_t cons_ddx_dx_index = cons_dx_x_index + size - 1;
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * size - 2, 3 * size);
    // x range.
    for (int i = 0; i < size; ++i) {
        cons(i, i) = 1;
    }
    // dx and x.
    for (int i = 0; i < size - 1; ++i) {
        cons(cons_dx_x_index + i, x_index + i + 1) = 1;
        cons(cons_dx_x_index + i, x_index + i) = -1;
        cons(cons_dx_x_index + i, dx_index + i) = -(layers_s_list_[i + 1] - layers_s_list_[i]);
    }
    // ddx and dx.
    for (int i = 0; i < size - 1; ++i) {
        cons(cons_ddx_dx_index + i, dx_index + i + 1) = 1;
        cons(cons_ddx_dx_index + i, dx_index + i) = -1;
        cons(cons_ddx_dx_index + i, ddx_index + i) = -(layers_s_list_[i + 1] - layers_s_list_[i]);
    }
    *matrix_constraints = cons.sparseView();

    // bounds.
    *lower_bound = Eigen::MatrixXd::Zero(3 * size - 2, 1);
    *upper_bound = Eigen::MatrixXd::Zero(3 * size - 2, 1);
    (*lower_bound)(0) = vehicle_l_wrt_smoothed_ref_;
    (*upper_bound)(0) = vehicle_l_wrt_smoothed_ref_;
    for (int i = 1; i < size; ++i) {
        (*lower_bound)(i) = layers_bounds_[i].first;
        (*upper_bound)(i) = layers_bounds_[i].second;
    }
}

ReferencePathSmoother::ReferencePathSmoother(const std::vector<State> &input_points,
                                             const State &start_state,
                                             const Map &grid_map) :
    input_points_(input_points),
    start_state_(start_state),
    grid_map_(grid_map) {}

inline double ReferencePathSmoother::getH(const APoint &p) const {
    // Note that this h is neither admissible nor consistent, so the result is not optimal.
    // There is a smoothing stage after this, so time efficiency is much more
    // important than optimality here.
    return (target_s_ - p.s) * 0.1;
//        return 0;
}
}

