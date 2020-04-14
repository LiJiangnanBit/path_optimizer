//
// Created by ljn on 20-4-14.
//
#include "glog/logging.h"
#include "path_optimizer/reference_path_smoother/angle_diff_smoother.hpp"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/config/planning_flags.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"

namespace PathOptimizationNS {

FgEvalFrenetSmooth::FgEvalFrenetSmooth(const std::vector<double> &seg_x_list,
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

void FgEvalFrenetSmooth::operator()(PathOptimizationNS::FgEvalFrenetSmooth::ADvector &fg,
                                    const PathOptimizationNS::FgEvalFrenetSmooth::ADvector &vars) {
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

AngleDiffSmoother::AngleDiffSmoother(const std::vector<PathOptimizationNS::State> &input_points,
                                     const PathOptimizationNS::State &start_state,
                                     const PathOptimizationNS::Map &grid_map)
    : ReferencePathSmoother(input_points, start_state, grid_map) {}

bool AngleDiffSmoother::smooth(PathOptimizationNS::ReferencePath *reference_path,
                               std::vector<PathOptimizationNS::State> *smoothed_path_display) {
    CHECK_GT(s_list_.size(), 2);
    CHECK_EQ(x_list_.size(), s_list_.size());
    CHECK_EQ(y_list_.size(), s_list_.size());
    tk::spline x_spline, y_spline;
    x_spline.set_points(s_list_, x_list_);
    y_spline.set_points(s_list_, y_list_);
    double max_s = s_list_.back();
    LOG(INFO) << "Ref path length: " << max_s;
    // Divide the reference path.
    double delta_beginning_s = 4;
    double delta_s = 3;
    std::vector<double> x_list, y_list, s_list, angle_list;
    s_list.push_back(0);
    s_list.push_back(delta_beginning_s);
    while (s_list.back() < max_s) {
        s_list.push_back(s_list.back() + delta_s);
    }
    if (max_s - s_list.back() > 1) {
        s_list.push_back(max_s);
    }
    auto N = s_list.size();
    // Store reference states in vectors. They will be used later.
    for (size_t i = 0; i != N; ++i) {
        double length_on_ref_path = s_list[i];
        double angle;
        angle = atan2(y_spline.deriv(1, length_on_ref_path), x_spline.deriv(1, length_on_ref_path));
        angle_list.push_back(angle);
        x_list.push_back(x_spline(length_on_ref_path));
        y_list.push_back(y_spline(length_on_ref_path));
    }

    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t n_vars = N;
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    // bounds of variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (size_t i = 0; i != n_vars; ++i) {
        vars_lowerbound[i] = -DBL_MAX;
        vars_upperbound[i] = DBL_MAX;
    }
    vars_lowerbound[0] = 0;
    vars_upperbound[0] = 0;
    vars_lowerbound[n_vars - 1] = 0;
    vars_upperbound[n_vars - 1] = 0;
    size_t n_constraints = 0;
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.1 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.1\n";
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    std::vector<double> weights;
    weights.push_back(FLAGS_frenet_angle_diff_weight); //curvature weight
    weights.push_back(FLAGS_frenet_angle_diff_diff_weight); //curvature rate weight
    weights.push_back(0.01); //distance to boundary weight
    weights.push_back(FLAGS_frenet_deviation_weight); //deviation weight
    FgEvalFrenetSmooth fg_eval_frenet(x_list,
                                      y_list,
                                      angle_list,
                                      s_list,
                                      N,
                                      weights);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalFrenetSmooth>(options, vars,
                                                     vars_lowerbound, vars_upperbound,
                                                     constraints_lowerbound, constraints_upperbound,
                                                     fg_eval_frenet, solution);
    // Check if it works
    if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
        LOG(WARNING) << "Angle diff smoother failed!";
        return false;
    }
    // output
    std::vector<double> result_x_list, result_y_list, result_s_list;
    double tmp_s = 0;
    for (size_t i = 0; i != N; i++) {
        double length_on_ref_path = s_list[i];
        double angle = angle_list[i];
        double new_angle = constraintAngle(angle + M_PI_2);
        double tmp_x = x_spline(length_on_ref_path) + solution.x[i] * cos(new_angle);
        double tmp_y = y_spline(length_on_ref_path) + solution.x[i] * sin(new_angle);
        result_x_list.emplace_back(tmp_x);
        result_y_list.emplace_back(tmp_y);
        if (i != 0) {
            tmp_s +=
                sqrt(pow(result_x_list[i] - result_x_list[i - 1], 2) + pow(result_y_list[i] - result_y_list[i - 1], 2));
        }
        result_s_list.emplace_back(tmp_s);
    }
    max_s = result_s_list.back();
    x_spline.set_points(result_s_list, result_x_list);
    y_spline.set_points(result_s_list, result_y_list);
    // Find the closest point to the vehicle.
    double min_dis_s = 0;
    double start_distance =
        sqrt(pow(start_state_.x - x_spline(0), 2) +
            pow(start_state_.y - y_spline(0), 2));
    if (start_distance > 0.001) {
        auto min_dis_to_vehicle = start_distance;
        double tmp_s_1 = 0.1;
        while (tmp_s_1 <= max_s) {
            double x = x_spline(tmp_s_1);
            double y = y_spline(tmp_s_1);
            double dis = sqrt(pow(x - start_state_.x, 2) + pow(y - start_state_.y, 2));
            if (dis <= min_dis_to_vehicle) {
                min_dis_to_vehicle = dis;
                min_dis_s = tmp_s_1;
            } else if (dis > 8 && min_dis_to_vehicle < 8) {
                break;
            }
            tmp_s_1 += 0.1;
        }
    }
    // Output. Start from the closest point from the vehicle.
    result_x_list.clear();
    result_y_list.clear();
    result_s_list.clear();
    double tmp_s_2 = min_dis_s;
    if (smoothed_path_display) smoothed_path_display->clear();
    while (tmp_s_2 <= max_s) {
        double x = x_spline(tmp_s_2);
        double y = y_spline(tmp_s_2);
        result_x_list.emplace_back(x);
        result_y_list.emplace_back(y);
        result_s_list.emplace_back(tmp_s_2 - min_dis_s);
        if (smoothed_path_display) {
            smoothed_path_display->emplace_back(x, y);
        }
        tmp_s_2 += 0.3;
    }
    tk::spline x_s_result, y_s_result;
    x_s_result.set_points(result_s_list, result_x_list);
    y_s_result.set_points(result_s_list, result_y_list);
    double max_s_result{result_s_list.back() + 3};
    reference_path->setSpline(x_s_result, y_s_result, max_s_result);
    LOG(INFO) << "Angle diff smoother succeeded!";
    return true;
}

}
