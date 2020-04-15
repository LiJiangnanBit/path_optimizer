//
// Created by ljn on 20-4-14.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_HPP_
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cfloat>
#include <tinyspline_ros/tinysplinecpp.h>
#include "path_optimizer/data_struct/data_struct.hpp"
#include "path_optimizer/reference_path_smoother/reference_path_smoother.hpp"
#include "path_optimizer/config/planning_flags.hpp"
#include "path_optimizer/tools/tools.hpp"

namespace PathOptimizationNS {

using CppAD::AD;
class FgEvalReferenceSmoothing {
 public:
    FgEvalReferenceSmoothing(const std::vector<double> &seg_x_list,
                             const std::vector<double> &seg_y_list,
                             const std::vector<double> &seg_s_list,
                             const std::vector<double> &seg_angle_list,
                             const int N) :
        N(N),
        seg_s_list_(seg_s_list),
        seg_x_list_(seg_x_list),
        seg_y_list_(seg_y_list),
        seg_angle_list_(seg_angle_list) {}
 public:
    size_t N{};
    const std::vector<double> &seg_s_list_;
    const std::vector<double> &seg_x_list_;
    const std::vector<double> &seg_y_list_;
    const std::vector<double> &seg_angle_list_;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef AD<double> ad;

    void operator()(ADvector &fg, const ADvector &vars) {
        for (size_t i = 1; i != N - 1; ++i) {
            ad last_offset = vars[i - 1];
            ad current_offset = vars[i];
            ad next_offset = vars[i + 1];
            ad last_x = seg_x_list_[i - 1] + last_offset * cos(seg_angle_list_[i - 1] + M_PI_2);
            ad last_y = seg_y_list_[i - 1] + last_offset * sin(seg_angle_list_[i - 1] + M_PI_2);
            ad current_x = seg_x_list_[i] + current_offset * cos(seg_angle_list_[i] + M_PI_2);
            ad current_y = seg_y_list_[i] + current_offset * sin(seg_angle_list_[i] + M_PI_2);
            ad next_x = seg_x_list_[i + 1] + next_offset * cos(seg_angle_list_[i + 1] + M_PI_2);
            ad next_y = seg_y_list_[i + 1] + next_offset * sin(seg_angle_list_[i + 1] + M_PI_2);
            ad ref_x = seg_x_list_[i];
            ad ref_y = seg_y_list_[i];
            // Deviation cost:
            fg[0] += FLAGS_cartesian_deviation_weight * (pow(current_offset, 2));
            // Curvature cost:
            fg[0] += FLAGS_cartesian_curvature_weight
                * (pow(next_x + last_x - 2 * current_x, 2) + pow(next_y + last_y - 2 * current_y, 2));
            // Set constraints.
            double cos_angle = cos(seg_angle_list_[i]);
            // offset.
            if (!isEqual(cos_angle, 0)) {
                fg[1 + i - 1] = (current_y - ref_y) / cos_angle;
            } else {
                fg[1 + i - 1] = -(current_x - ref_x) / sin(seg_angle_list_[i]);
            }
        }
    }
};

class TensionSmoother final : public ReferencePathSmoother {
 public:
    TensionSmoother() = delete;
    TensionSmoother(const std::vector<State> &input_points,
                    const State &start_state,
                    const Map &grid_map);
    ~TensionSmoother() override = default;

 private:
    bool smooth(PathOptimizationNS::ReferencePath *reference_path,
                std::vector<State> *smoothed_path_display) override;
};

}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_HPP_