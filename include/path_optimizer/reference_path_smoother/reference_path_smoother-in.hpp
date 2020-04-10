//
// Created by ljn on 20-4-9.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_IN_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_IN_HPP_

#include "path_optimizer/data_struct/reference_path.hpp"

namespace PathOptimizationNS {
template<typename Smoother>
bool ReferencePathSmoother::solve(PathOptimizationNS::ReferencePath *reference_path,
                                  std::vector<State> *smoothed_path_display) {
    bSpline();
    if (config_.modify_input_points_) {
        if (modifyInputPoints()) {
            // If searching process succeeded, add the searched result into reference_path.
            tk::spline searched_xs, searched_ys;
            searched_xs.set_points(s_list_, x_list_);
            searched_ys.set_points(s_list_, y_list_);
            reference_path->setOriginalSpline(searched_xs, searched_ys, s_list_.back());
        }
    }
    Smoother smoother(x_list_, y_list_, s_list_, start_state_, grid_map_, config_);
    return smoother.smooth(reference_path, smoothed_path_display);
}
}

#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_IN_HPP_
