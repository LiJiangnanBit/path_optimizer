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
#include "path_optimizer/reference_path_smoother/qp_smoother.hpp"

namespace PathOptimizationNS {

std::unique_ptr<ReferencePathSmoother> ReferencePathSmoother::create(std::string &type,
                                                                     const std::vector<State> &input_points,
                                                                     const State &start_state,
                                                                     const Map &grid_map) {
    if (type == "ANGLE_DIFF") {
        return std::unique_ptr<ReferencePathSmoother>{new AngleDiffSmoother(input_points, start_state, grid_map)};
    } else if (type == "TENSION") {
        return std::unique_ptr<ReferencePathSmoother>{new QPSmoother(input_points, start_state, grid_map)};
    } else {
        LOG(ERROR) << "No such smoother!";
        return nullptr;
    }
}

bool ReferencePathSmoother::solve(PathOptimizationNS::ReferencePath *reference_path,
                                  std::vector<PathOptimizationNS::State> *smoothed_path_display) {
    bSpline();
    if (FLAGS_enable_searching && modifyInputPoints()) {
        // If searching process succeeded, add the searched result into reference_path.
        tk::spline searched_xs, searched_ys;
        searched_xs.set_points(s_list_, x_list_);
        searched_ys.set_points(s_list_, y_list_);
        reference_path->setOriginalSpline(searched_xs, searched_ys, s_list_.back());
    }
    return smooth(reference_path, smoothed_path_display);
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
    std::cout << "ref path length: " << max_s << std::endl;
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

double ReferencePathSmoother::getClosestPointOnSpline(const PathOptimizationNS::tk::spline &x_s,
                                                      const PathOptimizationNS::tk::spline &y_s,
                                                      const double max_s) const {
    // Find the closest point to the vehicle.
    double min_dis_s = 0;
    double start_distance =
        sqrt(pow(start_state_.x - x_s(0), 2) +
            pow(start_state_.y - y_s(0), 2));
    if (!isEqual(start_distance, 0)) {
        auto min_dis_to_vehicle = start_distance;
        double tmp_s_1 = 0 + 0.1;
        while (tmp_s_1 <= max_s) {
            double x = x_s(tmp_s_1);
            double y = y_s(tmp_s_1);
            double dis = sqrt(pow(x - start_state_.x, 2) + pow(y - start_state_.y, 2));
            if (dis <= min_dis_to_vehicle) {
                min_dis_to_vehicle = dis;
                min_dis_s = tmp_s_1;
            } else if (dis > 15 && min_dis_to_vehicle < 15) {
                break;
            }
            tmp_s_1 += 0.1;
        }
    }
    return min_dis_s;
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
    double offset_cost = fabs(point.offset) / FLAGS_search_lateral_range * FLAGS_search_deviation_cost;
    // Smoothness cost.
//    double smoothness_cost = 0;
//    if (parent.parent) {
//        Eigen::Vector2d v1(parent.x - parent.parent->x, parent.y - parent.parent->y);
//        Eigen::Vector2d v2(point.x - parent.x, point.y - parent.y);
//        smoothness_cost = fabs(v1(0) * v2(1) - v1(1) * v2(0)) * SMOOTHNESS_COST;
//    }
//    printExp(offset_cost);
//    printExp(smoothness_cost);
//    printExp(obstacle_cost);
//    return parent.g + offset_cost + smoothness_cost + obstacle_cost;
    return parent.g + offset_cost + obstacle_cost;
}

bool ReferencePathSmoother::modifyInputPoints() {
    auto t1 = std::clock();
    if (x_list_.empty() || y_list_.empty() || s_list_.empty()) return false;
    tk::spline x_s, y_s;
    x_s.set_points(s_list_, x_list_);
    y_s.set_points(s_list_, y_list_);
    // Sampling interval.
    double tmp_s = 0;
    std::vector<double> layers_s_list;
    while (tmp_s < s_list_.back()) {
        layers_s_list.emplace_back(tmp_s);
        tmp_s += FLAGS_search_longitudial_spacing;
    }
    layers_s_list.emplace_back(s_list_.back());
    target_s_ = layers_s_list.back();

    // Sample points.
    APoint start_point;
    start_point.x = x_s(0);
    start_point.y = y_s(0);
    start_point.s = 0;
    start_point.l = 0;
    start_point.layer = 0;
    start_point.g = 0;
    start_point.h = getH(start_point);
    sampled_points_.emplace_back(std::vector<APoint>{start_point});
    for (size_t i = 1; i != layers_s_list.size(); ++i) {
        double sr = layers_s_list[i];
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
        while (offset <= left_range) {
            APoint point;
            point.s = sr;
            point.l = offset;
            point.x = xr + offset * cos(hr + M_PI_2);
            point.y = yr + offset * sin(hr + M_PI_2);
            point.layer = i;
            point.offset = offset;
            grid_map::Position position(point.x, point.y);
            if (grid_map_.isInside(position)
                && grid_map_.getObstacleDistance(position) > FLAGS_circle_radius) {
                point_set.emplace_back(point);
            }
            offset += FLAGS_search_lateral_spacing;
        }
        sampled_points_.emplace_back(point_set);
    }

    // Push the start point into the open set.
    open_set_.push(&sampled_points_[0][0]);
    sampled_points_[0][0].is_in_open_set = true;

    // Search.
    while (true) {
        if (open_set_.empty()) {
            LOG(WARNING) << "Lattice search failed!";
            return false;
        }
        auto tmp_point_ptr = open_set_.top();
        if (isEqual(tmp_point_ptr->s, target_s_)) {
            break;
        }
        open_set_.pop();
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
    std::vector<double> a_x_list, a_y_list;
    auto ptr = open_set_.top();
    while (ptr) {
        a_x_list.emplace_back(ptr->x);
        a_y_list.emplace_back(ptr->y);
        ptr = ptr->parent;
    }
    std::reverse(a_x_list.begin(), a_x_list.end());
    std::reverse(a_y_list.begin(), a_y_list.end());

    // B spline fitting.
    // Choose a control point every n points, interval being 4.5m.
    auto n = std::max(static_cast<int>(4.5 / FLAGS_search_longitudial_spacing), 1);
    int control_points_num = (a_x_list.size() - 1) / n + 1;
    int degree = 3;
    if (control_points_num <= degree) {
        LOG(WARNING) << "Reference path is too short for BSpline!";
        return false;
    }
    // Fit.
    tinyspline::BSpline b_spline(control_points_num, 2, degree);
    std::vector<tinyspline::real> ctrlp = b_spline.controlPoints();
    for (size_t i = 0; i != control_points_num - 1; ++i) {
        ctrlp[2 * (i)] = a_x_list[i * n];
        ctrlp[2 * (i) + 1] = a_y_list[i * n];
    }
    // The last point.
    ctrlp[2 * (control_points_num - 1)] = a_x_list.back();
    ctrlp[2 * (control_points_num - 1) + 1] = a_y_list.back();
    b_spline.setControlPoints(ctrlp);

    // Store the results for further smoothing.
    x_list_.clear();
    y_list_.clear();
    s_list_.clear();
    double delta_t = 1.0 / target_s_;
    double tmp_t = 0;
    while (tmp_t < 1) {
        auto result = b_spline.eval(tmp_t).result();
        x_list_.emplace_back(result[0]);
        y_list_.emplace_back(result[1]);
        tmp_t += delta_t;
    }
    // The last point.
    auto result = b_spline.eval(1).result();
    x_list_.emplace_back(result[0]);
    y_list_.emplace_back(result[1]);
    // Get s.
    s_list_.emplace_back(0);
    for (size_t i = 1; i != x_list_.size(); ++i) {
        double dis = sqrt(pow(x_list_[i] - x_list_[i - 1], 2) + pow(y_list_[i] - y_list_[i - 1], 2));
        s_list_.emplace_back(s_list_.back() + dis);
    }
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

