//
// Created by ljn on 20-1-31.
//

#ifndef PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_CARTESIAN_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_CARTESIAN_REFERENCE_PATH_SMOOTHER_HPP_

#include <vector>
#include <opt_utils/opt_utils.hpp>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <tinyspline_ros/tinysplinecpp.h>
#include <internal_grid_map/internal_grid_map.hpp>
#include <path_optimizer/path_optimizer.hpp>
#include "data_struct/data_struct.hpp"
#include "config/config.hpp"
#include "tools/spline.h"
#include "tools/tools.hpp"

namespace PathOptimizationNS {

using CppAD::AD;
class FgEvalReferenceSmoothing {
public:
    FgEvalReferenceSmoothing(const std::vector<double> &seg_x_list,
                             const std::vector<double> &seg_y_list,
                             const std::vector<double> &seg_s_list,
                             const int &N) :
        N(N),
        seg_s_list_(seg_s_list),
        seg_x_list_(seg_x_list),
        seg_y_list_(seg_y_list),
        curvature_weight_(10.0),
        deviation_weight_(0.001) {}
public:
    size_t N;
    const std::vector<double> &seg_s_list_;
    const std::vector<double> &seg_x_list_;
    const std::vector<double> &seg_y_list_;
    const double curvature_weight_, deviation_weight_;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef AD<double> ad;

    void operator()(ADvector &fg, const ADvector &vars) {
        const auto x_range_begin = 0;
        const auto y_range_begin = x_range_begin + N;
        for (size_t i = 1; i != N - 1; ++i) {
            ad last_x = vars[x_range_begin + i - 1];
            ad last_y = vars[y_range_begin + i - 1];
            ad current_x = vars[x_range_begin + i];
            ad current_y = vars[y_range_begin + i];
            ad next_x = vars[x_range_begin + i + 1];
            ad next_y = vars[y_range_begin + i + 1];
            ad ref_x = seg_x_list_[i];
            ad ref_y = seg_y_list_[i];
            // Deviation cost:
            fg[0] += deviation_weight_ * (pow(current_x - ref_x, 2) + pow(current_y - ref_y, 2));
            // Curvature cost:
            fg[0] += curvature_weight_
                * (pow(next_x + last_x - 2 * current_x, 2) + pow(next_y + last_y - 2 * current_y, 2));
            // Set constraints.
            fg[1 + i - 1] = pow(current_x - ref_x, 2) + pow(current_y - ref_y, 2);
        }
        // The last point:
        fg[0] += deviation_weight_ * (pow(vars[x_range_begin + N - 1] - seg_x_list_.back(), 2)
            + pow(vars[y_range_begin + N - 1] - seg_y_list_.back(), 2));
        fg[N - 2] = pow(vars[x_range_begin + N - 1] - seg_x_list_.back(), 2)
            + pow(vars[y_range_begin + N - 1] - seg_y_list_.back(), 2);
    }
};

class CartesianReferencePathSmoother {

public:

    CartesianReferencePathSmoother() = delete;

    CartesianReferencePathSmoother(const std::vector<double> &x_list,
                                   const std::vector<double> &y_list,
                                   const std::vector<double> &s_list,
                                   const hmpl::State &start_state,
                                   const hmpl::InternalGridMap &grid_map,
                                   const Config &config);

    // Core function.
    bool smooth(ReferencePath *reference_path, std::vector<hmpl::State> *smoothed_path_display = nullptr) const;

private:

    // Smoothing in Cartesian frame. Much slower than the other method.
    bool smoothPathCartesian(tk::spline *x_s_out,
                             tk::spline *y_s_out,
                             double *max_s_out,
                             std::vector<hmpl::State> *smoothed_path_display) const;

    const std::vector<double> &x_list_, &y_list_, &s_list_;
    const hmpl::State &start_state_;
    const hmpl::InternalGridMap &grid_map_;
    const Config &config_;
};
}

#endif //PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_CARTESIAN_REFERENCE_PATH_SMOOTHER_HPP_
