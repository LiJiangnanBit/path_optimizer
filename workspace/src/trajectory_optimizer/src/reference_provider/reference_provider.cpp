//
// Created by ljn on 20-2-20.
//

#include <cmath>
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

bool ReferenceProvider::getReferenceTrajectory(const std::shared_ptr<FrenetTrajectory> &reference_trajectory) {
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

    // Get reference speed.
    double tmp_s{0}, tmp_v{start_state_.v}, tmp_k{start_state_.k};
    reference_trajectory->state_list.clear();
    while (tmp_s <= std::min(TrajOptConfig::max_length_, ref_s_list.back())) {
//        printf("s: %f, k: %f, v: %f\n", tmp_s, tmp_k, tmp_v);
        reference_trajectory->state_list.emplace_back(0, 0, tmp_s, tmp_k, tmp_v);
        // Update;
        tmp_s += TrajOptConfig::spacing_;
        tmp_k = getCurvature(reference_trajectory->x_s, reference_trajectory->y_s, tmp_s);
        double next_max_v{sqrt(TrajOptConfig::max_lat_acc_ / std::max(0.01, fabs(tmp_k)))};
        next_max_v = std::min(next_max_v, TrajOptConfig::max_v_);
        double time_interval = (TrajOptConfig::spacing_ / std::max(tmp_v, 1.0));
        double acc{(next_max_v - tmp_v) / time_interval};
        if (acc > TrajOptConfig::max_lon_acc_) {
            tmp_v = std::sqrt(pow(tmp_v, 2) + 2 * TrajOptConfig::max_lon_acc_ * TrajOptConfig::spacing_);
        } else if (acc < TrajOptConfig::max_lon_dacc_) {
            tmp_v = std::sqrt(pow(tmp_v, 2) + 2 * TrajOptConfig::max_lon_dacc_ * TrajOptConfig::spacing_);
        } else {
            tmp_v = next_max_v;
        }
    }
    reference_trajectory->path_length_ = reference_trajectory->state_list.back().s;
    reference_trajectory->spacing = TrajOptConfig::spacing_;
    // Calculate other elements.
    for (size_t i = 0; i != reference_trajectory->state_list.size(); ++i) {
        auto &state{reference_trajectory->state_list[i]};
        state.ay = std::pow(state.v, 2) * state.k;
        if (i != reference_trajectory->state_list.size() - 1) {
            double next_v{reference_trajectory->state_list[i + 1].v};
            double next_k{reference_trajectory->state_list[i + 1].k};
            state.vp = (next_v - state.v) / TrajOptConfig::spacing_; // TODO: initial vp?
            state.kp = (next_k - state.k) / TrajOptConfig::spacing_;
        } else {
            state.vp = state.kp = 0;
        }
    }
    return true;
}
}