//
// Created by ljn on 20-1-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_

#include <vector>
#include <opt_utils/opt_utils.hpp>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <tinyspline_ros/tinysplinecpp.h>
#include <internal_grid_map/internal_grid_map.hpp>
#include <path_optimizer/path_optimizer.hpp>
#include "config/config.hpp"
#include "tools/spline.h"
#include "tools/tools.hpp"
#include "FgEvalReferenceSmoothingFrenet.hpp"
#include "FgEvalReferenceSmoothing.hpp"

namespace PathOptimizationNS {

struct ReferencePath {
    ReferencePath() = default;
    // Copy some elements from another one.
    ReferencePath(const ReferencePath &divided_segments_, size_t target_index) :
        x_s_(divided_segments_.x_s_),
        y_s_(divided_segments_.y_s_),
        max_s_(divided_segments_.max_s_) {
        assert(target_index <= divided_segments_.seg_angle_list_.size());
        seg_s_list_.assign(divided_segments_.seg_s_list_.begin(), divided_segments_.seg_s_list_.begin() + target_index);
        seg_angle_list_.assign(divided_segments_.seg_angle_list_.begin(),
                               divided_segments_.seg_angle_list_.begin() + target_index);
        seg_k_list_.assign(divided_segments_.seg_k_list_.begin(), divided_segments_.seg_k_list_.begin() + target_index);
        seg_clearance_list_.assign(divided_segments_.seg_clearance_list_.begin(),
                                   divided_segments_.seg_clearance_list_.begin() + target_index);
        seg_x_list_.assign(divided_segments_.seg_x_list_.begin(), divided_segments_.seg_x_list_.begin() + target_index);
        seg_y_list_.assign(divided_segments_.seg_y_list_.begin(), divided_segments_.seg_y_list_.begin() + target_index);
    }
    // Reference path representation.
    tk::spline x_s_;
    tk::spline y_s_;
    double max_s_;
    // Divided smoothed path info.
    std::vector<double> seg_s_list_;
    std::vector<double> seg_k_list_;
    std::vector<double> seg_x_list_;
    std::vector<double> seg_y_list_;
    std::vector<double> seg_angle_list_;
    std::vector<std::vector<double> > seg_clearance_list_;
};

class ReferencePathSmoother {
public:
    ReferencePathSmoother() = delete;
    explicit ReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                                   const hmpl::State &start_state,
                                   const hmpl::InternalGridMap &grid_map,
                                   const Config &config);
    bool smooth(ReferencePath *reference_path, std::vector<hmpl::State> *smoothed_path_display = nullptr) const;

private:
    bool smoothPathFrenet(tk::spline *x_s_out,
                          tk::spline *y_s_out,
                          double *max_s_out,
                          std::vector<hmpl::State> *smoothed_path_display) const;
    bool smoothPathCartesian(tk::spline *x_s_out,
                             tk::spline *y_s_out,
                             double *max_s_out,
                             std::vector<hmpl::State> *smoothed_path_display) const;
    const std::vector<hmpl::State> &points_list_;
    const hmpl::State start_state_;
    const hmpl::InternalGridMap grid_map_;
    const Config &config_;
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
