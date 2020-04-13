//
// Created by ljn on 20-1-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cfloat>
#include <tinyspline_ros/tinysplinecpp.h>
#include "../data_struct/data_struct.hpp"

namespace PathOptimizationNS {

class Map;
class ReferencePath;
namespace tk {
class spline;
}

using CppAD::AD;
class FgEvalFrenetSmooth {
public:
    FgEvalFrenetSmooth(const std::vector<double> &seg_x_list,
                       const std::vector<double> &seg_y_list,
                       const std::vector<double> &seg_angle_list,
                       const std::vector<double> &seg_s_list,
                       const int &N,
                       const std::vector<double> &cost_func) :
        N(N),
        seg_s_list_(seg_s_list),
        seg_x_list_(seg_x_list),
        seg_y_list_(seg_y_list),
        seg_angle_list_(seg_angle_list),
        cost_func_curvature_weight_(cost_func[0]),
        cost_func_curvature_rate_weight_(cost_func[1]),
        cost_func_bound_weight_(cost_func[2]),
        cost_func_s_weight_(cost_func[3]) {}
public:
    size_t N;
    const std::vector<double> &seg_s_list_;
    const std::vector<double> &seg_x_list_;
    const std::vector<double> &seg_y_list_;
    const std::vector<double> &seg_angle_list_;
    double cost_func_curvature_weight_{};
    double cost_func_curvature_rate_weight_{};
    double cost_func_bound_weight_{};
    double cost_func_s_weight_{};

    typedef CPPAD_TESTVECTOR(AD <double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        // The rest of the constraints
        AD<double> curvature_by_position_before;
        AD<double> curvature_by_position;
        AD<double> heading;
        AD<double> heading_before;
        for (size_t i = 2; i != N; ++i) {
            AD<double> pq = vars[i];
            AD<double> pq_before_before;
            AD<double> pq_before;
            AD<double> ref_x_before_before;
            AD<double> ref_y_before_before;
            AD<double> ref_angle_before_before;
            AD<double> ref_x_before;
            AD<double> ref_y_before;
            AD<double> ref_angle_before;
            AD<double> ref_x;
            AD<double> ref_y;
            AD<double> ref_angle;
            AD<double> x_before_before;
            AD<double> y_before_before;
            AD<double> x_before;
            AD<double> y_before;
            AD<double> x;
            AD<double> y;
            pq_before_before = vars[i - 2];
            pq_before = vars[i - 1];
            ref_x_before_before = seg_x_list_[i - 2];
            ref_y_before_before = seg_y_list_[i - 2];
            ref_angle_before_before = seg_angle_list_[i - 2];
            ref_x_before = seg_x_list_[i - 1];
            ref_y_before = seg_y_list_[i - 1];
            ref_angle_before = seg_angle_list_[i - 1];
            ref_x = seg_x_list_[i];
            ref_y = seg_y_list_[i];
            ref_angle = seg_angle_list_[i];
            x_before_before = ref_x_before_before + pq_before_before * CppAD::cos(ref_angle_before_before + M_PI_2);
            y_before_before = ref_y_before_before + pq_before_before * CppAD::sin(ref_angle_before_before + M_PI_2);
            x_before = ref_x_before + pq_before * CppAD::cos(ref_angle_before + M_PI_2);
            y_before = ref_y_before + pq_before * CppAD::sin(ref_angle_before + M_PI_2);
            x = ref_x + pq * CppAD::cos(ref_angle + M_PI_2);
            y = ref_y + pq * CppAD::sin(ref_angle + M_PI_2);
            if (seg_x_list_[i] - seg_x_list_[i - 1] < 0) {
                heading = CppAD::atan2(-y + y_before, -x + x_before);
                heading_before = CppAD::atan2(-y_before + y_before_before, -x_before + x_before_before);
                curvature_by_position = heading - heading_before;
            } else {
                heading = CppAD::atan2(y - y_before, x - x_before);
                heading_before = CppAD::atan2(y_before - y_before_before, x_before - x_before_before);
                curvature_by_position = (heading - heading_before);
            }
            fg[0] += cost_func_curvature_weight_ * pow(curvature_by_position, 2);
            fg[0] +=
                cost_func_curvature_rate_weight_ * pow(curvature_by_position - curvature_by_position_before, 2);
            fg[0] += cost_func_s_weight_ * pow(pq, 2);
            curvature_by_position_before = curvature_by_position;
        }
        fg[0] += pow(vars[N - 2], 2) + pow(vars[N - 1], 2);
    }
};

class FrenetReferencePathSmoother {

public:

    FrenetReferencePathSmoother() = delete;

    FrenetReferencePathSmoother(const std::vector<double> &x_list,
                                const std::vector<double> &y_list,
                                const std::vector<double> &s_list,
                                const State &start_state,
                                const Map &grid_map);

    // Core function.
    bool smooth(ReferencePath *reference_path, std::vector<State> *smoothed_path_display = nullptr) const;

private:

    // Smoothing in frenet frame.
    bool smoothPathFrenet(tk::spline *x_s_out,
                          tk::spline *y_s_out,
                          double *max_s_out,
                          std::vector<State> *smoothed_path_display) const;

    const std::vector<double> &x_list_, &y_list_, &s_list_;
    const State &start_state_;
    const Map &grid_map_;
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
