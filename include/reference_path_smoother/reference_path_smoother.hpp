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
#include "config/config.hpp"
#include "tools/spline.h"
#include "tools/tools.hpp"
#include "FgEvalReferenceSmoothingFrenet.hpp"
#include "FgEvalReferenceSmoothing.hpp"

namespace PathOptimizationNS {

class ReferencePathSmoother {
public:
    ReferencePathSmoother() = delete;
    explicit ReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                                   const hmpl::State &start_state,
                                   const hmpl::InternalGridMap &grid_map,
                                   const Config &config);
    bool smooth(tk::spline *x_s_out,
                tk::spline *y_s_out,
                double *max_s_out,
                std::vector<hmpl::State> *smoothed_path_display = nullptr) const;

    bool testFcn(double i) ;
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
