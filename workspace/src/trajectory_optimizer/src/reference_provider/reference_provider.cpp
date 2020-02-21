//
// Created by ljn on 20-2-20.
//

#include <path_optimizer/tools/tools.hpp>
#include "trajectory_optimizer/reference_provider/reference_provider.hpp"

using PathOptimizationNS::distance;
using PathOptimizationNS::getHeading;
using PathOptimizationNS::getCurvature;

namespace TrajOptNS {

ReferenceProvider::ReferenceProvider(const std::vector<State> &points_list,
                                     const State &start_state,
                                     const State &end_state,
                                     const grid_map::GridMap &map)
    : points_list_(points_list),
      start_state_(start_state),
      end_state_(end_state),
      grid_map_(map) {}

bool ReferenceProvider::getReferenceTrajectory(TrajOptNS::Trajectory *reference_trajectory) {
    // Get reference path.
    std::vector<State> reference_path;
    PathOptimizationNS::PathOptimizer path_optimizer(points_list_, start_state_, end_state_, grid_map_);
    path_optimizer.setConfig("raw_result_", true);
    path_optimizer.setConfig("output_interval_", 1.2);
    if (!path_optimizer.solve(&reference_path)) {
        return false;
    }
    std::vector<double> ref_x_list, ref_y_list, ref_s_list;
    double tmp_ref_s{0};
    for (size_t i{0}; i != reference_path.size(); ++i) {
        if (i != 0) {
            tmp_ref_s += distance(reference_path[i - 1], reference_path[i]);
        }
        ref_s_list.emplace_back(tmp_ref_s);
        ref_x_list.emplace_back(reference_path[i].x);
        ref_y_list.emplace_back(reference_path[i].y);
    }
    reference_trajectory->x_s.set_points(ref_s_list, ref_x_list);
    reference_trajectory->y_s.set_points(ref_s_list, ref_y_list);
    reference_trajectory->path_length_ = ref_s_list.back();

    // Get reference speed.
    reference_trajectory->time_interval = TrajOptConfig::time_interval_;
    reference_trajectory->max_time = TrajOptConfig::horizon_time_;
    double tmp_t{0}, tmp_s{0}, tmp_v{std::max(start_state_.v, 0.1)};
    while (tmp_t <= TrajOptConfig::horizon_time_) {
        double x{reference_trajectory->x_s(tmp_s)};
        double y{reference_trajectory->y_s(tmp_s)};
        double z{getHeading(reference_trajectory->x_s, reference_trajectory->y_s, tmp_s)};
        double k{getCurvature(reference_trajectory->x_s, reference_trajectory->y_s, tmp_s)};
        reference_trajectory->state_list.emplace_back(x, y, z, k, tmp_s, tmp_v);
        tmp_t += TrajOptConfig::time_interval_;
        tmp_s += tmp_v * TrajOptConfig::time_interval_;
        if (tmp_s > reference_trajectory->path_length_) break;
        double next_curvature{getCurvature(reference_trajectory->x_s, reference_trajectory->y_s, tmp_s)};
        double next_max_v{sqrt(TrajOptConfig::max_lat_acc_ / std::max(0.01, fabs(next_curvature)))};
        next_max_v = std::min(next_max_v, TrajOptConfig::max_v_);
        double acc{(next_max_v - tmp_v) / TrajOptConfig::time_interval_};
        if (acc > TrajOptConfig::max_lon_acc_) {
            tmp_v += TrajOptConfig::max_lon_acc_ * TrajOptConfig::time_interval_;
        } else if (acc < TrajOptConfig::max_lon_dacc_) {
            tmp_v += TrajOptConfig::max_lon_dacc_ * TrajOptConfig::time_interval_;
        } else {
            tmp_v = next_max_v;
        }
    }
    return true;
}
}