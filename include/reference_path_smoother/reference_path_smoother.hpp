//
// Created by ljn on 20-1-31.
//

#ifndef PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <opt_utils/opt_utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <ctime>
#include "config/config.hpp"
#include "data_struct/data_struct.hpp"
#include "tools/spline.h"
#include "tools/tools.hpp"

namespace PathOptimizationNS {
#define OBSTACLE_COST 0.5

// This class use A* search to improve the quality of the input points(if needed), and
// then uses a smoother to obtain a smoothed reference path.
template<typename Smoother>
class ReferencePathSmoother {

public:
    ReferencePathSmoother() = delete;
    ReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                          const hmpl::State &start_state,
                          const hmpl::InternalGridMap &grid_map,
                          const Config &config);

    bool solve(ReferencePath *reference_path, std::vector<hmpl::State> *smoothed_path_display = nullptr);
    std::vector<std::vector<double>> display() const {
        return std::vector<std::vector<double>>{x_list_, y_list_, s_list_};
    }

private:
    void bSpline();
    // A* search.
    bool modifyInputPoints();
    bool checkExistenceInClosedSet(const APoint &point) const;
    double getG(const APoint &point, const APoint &parent) const;
    inline double getH(const APoint &p) {
        return target_s_ - p.s;
    }

    const std::vector<hmpl::State> &input_points_;
    const hmpl::State &start_state_;
    const hmpl::InternalGridMap &grid_map_;
    const Config &config_;
    // Data to be passed into solvers.
    std::vector<double> x_list_, y_list_, s_list_;
    // Sampled points.
    std::vector<std::vector<APoint>> sampled_points_;
    double target_s_ = 0;
    std::priority_queue<APoint*, std::vector<APoint*>, PointComparator> open_set_;
    std::vector<APoint*> closed_set_;

};

template<typename Smoother>
double ReferencePathSmoother<Smoother>::getG(const PathOptimizationNS::APoint &point,
                                           const PathOptimizationNS::APoint &parent) const {
    double distance = sqrt(pow(point.x - parent.x, 2) + pow(point.y - parent.y, 2));
    grid_map::Position position(point.x, point.y);
    double obstacle_cost = 0;
    double distance_to_obs = grid_map_.getObstacleDistance(position);
    double safety_distance = 5;
    if (distance_to_obs < safety_distance) {
        obstacle_cost = (safety_distance - distance_to_obs) / safety_distance * OBSTACLE_COST;
    }
    return parent.g + distance + obstacle_cost;
}

template<typename Smoother>
bool ReferencePathSmoother<Smoother>::modifyInputPoints() {
    auto t1 = std::clock();
    if (x_list_.empty() || y_list_.empty() || s_list_.empty()) return false;
    tk::spline x_s, y_s;
    x_s.set_points(s_list_, x_list_);
    y_s.set_points(s_list_, y_list_);
    // Sampling interval.
    double tmp_s = 0;
    std::vector<double> layers_s_list;
    while (tmp_s <= s_list_.back()) {
        layers_s_list.emplace_back(tmp_s);
        tmp_s += config_.a_star_longitudinal_interval_;
    }
    if (s_list_.back() - layers_s_list.back() > 0.3) layers_s_list.emplace_back(s_list_.back());
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
        std::vector<APoint> point_set;
        double offset = -config_.a_star_lateral_range_;
        while (offset <= config_.a_star_lateral_range_ ) {
            APoint point;
            point.s = sr;
            point.l = offset;
            point.x = xr + offset * cos(hr + M_PI_2);
            point.y = yr + offset * sin(hr + M_PI_2);
            point.layer = i;
            grid_map::Position position(point.x, point.y);
            if (!grid_map_.maps.isInside(position)
                || grid_map_.getObstacleDistance(position) < config_.circle_radius_) {
                offset += config_.a_star_lateral_interval_;
                continue;
            }
            point_set.emplace_back(point);
            offset += config_.a_star_lateral_interval_;
        }
        sampled_points_.emplace_back(point_set);
    }

    // Push the start point into the open set.
    open_set_.push(&sampled_points_[0][0]);
    sampled_points_[0][0].is_in_open_set = true;

    // Search.
    while (true) {
        if (open_set_.empty()) {
            std::cout << "a* fail" << std::endl;
            return false;
        }
        auto tmp_point_ptr = open_set_.top();
        if (isEqual(tmp_point_ptr->s, target_s_)) {
            std::cout << "a* reached target" << std::endl;
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
        closed_set_.emplace_back(tmp_point_ptr);
    }

    // Retrieve optimal path.
    std::vector<double> a_x_list, a_y_list;
    auto ptr = open_set_.top();
    while (ptr) {
        a_x_list.emplace_back(ptr->x);
        a_y_list.emplace_back(ptr->y);
        ptr = ptr->parent;
    }
    std::reverse(a_x_list.begin(), a_x_list.end());
    std::reverse(a_y_list.begin(), a_y_list.end());
    
    // Modify.
    tinyspline::BSpline b_spline(a_x_list.size(), 2, 3);
    std::vector<tinyspline::real> ctrlp = b_spline.controlPoints();
    for (size_t i = 0; i != a_x_list.size(); ++i) {
        ctrlp[2 * (i)] = a_x_list[i];
        ctrlp[2 * (i) + 1] = a_y_list[i];
    }
    b_spline.setControlPoints(ctrlp);
    x_list_.clear();
    y_list_.clear();
    s_list_.clear();
    double delta_t = 1.0 / a_x_list.size();
    double tmp_t = 0;
    while (tmp_t <= 1) {
        auto result = b_spline.eval(tmp_t).result();
        x_list_.emplace_back(result[0]);
        y_list_.emplace_back(result[1]);
        tmp_t += delta_t;
    }
    s_list_.emplace_back(0);
    for (size_t i = 1; i != x_list_.size(); ++i) {
        double dis = sqrt(pow(x_list_[i] - x_list_[i - 1], 2) + pow(y_list_[i] - y_list_[i - 1], 2));
        s_list_.emplace_back(s_list_.back() + dis);
    }
    auto t2 = std::clock();
    std::cout << "a* time cost: " << time_s(t1, t2) << std::endl;
    return true;
}

template<typename Smoother>
bool ReferencePathSmoother<Smoother>::checkExistenceInClosedSet(const APoint &point) const{
    for (const auto &iter : closed_set_) {
        if (iter == &point) {
            return true;
        }
//        if (isEqual(iter->s, point.s) && isEqual(iter->l, point.l)) {
//            return true;
//        }
    }
    return false;
}

template<typename Smoother>
void ReferencePathSmoother<Smoother>::bSpline() {
    // B spline smoothing.
    double length = 0;
    for (size_t i = 0; i != input_points_.size() - 1; ++i) {
        length += hmpl::distance(input_points_[i], input_points_[i + 1]);
    }
    int degree = 3;
    double average_length = length / input_points_.size();
    if (average_length > 10) degree = 3;
    else if (average_length > 5) degree = 4;
    else degree = 5;
    std::cout << "b spline degree: " << degree << std::endl;
    tinyspline::BSpline b_spline_raw(input_points_.size(), 2, degree);
    std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
    for (size_t i = 0; i != input_points_.size(); ++i) {
        ctrlp_raw[2 * (i)] = input_points_[i].x;
        ctrlp_raw[2 * (i) + 1] = input_points_[i].y;
    }
    b_spline_raw.setControlPoints(ctrlp_raw);
    double delta_t = 1.0 / length;
    double tmp_t = 0;
    while (tmp_t <= 1) {
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

template<typename Smoother>
ReferencePathSmoother<Smoother>::ReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                                                       const hmpl::State &start_state,
                                                       const hmpl::InternalGridMap &grid_map,
                                                       const Config &config) :
    input_points_(input_points),
    start_state_(start_state),
    grid_map_(grid_map),
    config_(config) {}

template<typename Smoother>
bool ReferencePathSmoother<Smoother>::solve(PathOptimizationNS::ReferencePath *reference_path,
                                            std::vector<hmpl::State> *smoothed_path_display) {
    bSpline();
    if (config_.modify_input_points_) {
        modifyInputPoints();
    }
    Smoother smoother(x_list_, y_list_, s_list_, start_state_, grid_map_, config_);
    return smoother.smooth(reference_path, smoothed_path_display);
}
}

#endif //PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
