//
// Created by ljn on 20-5-4.
//
#include <tinyspline_ros/tinysplinecpp.h>
#include "OsqpEigen/OsqpEigen.h"
#include "glog/logging.h"
#include "path_optimizer/tools/Map.hpp"
#include "path_optimizer/reference_path_smoother/qp_smoother.hpp"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/config/planning_flags.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"

namespace PathOptimizationNS {

FgEvalQPSmoothing::FgEvalQPSmoothing(const std::vector<double> &seg_x_list,
                                     const std::vector<double> &seg_y_list,
                                     const std::vector<double> &seg_s_list,
                                     const std::vector<double> &seg_angle_list) :
    FgEvalReferenceSmoothing(seg_x_list, seg_y_list, seg_s_list, seg_angle_list) {}

void FgEvalQPSmoothing::operator()(PathOptimizationNS::FgEvalReferenceSmoothing::ADvector &fg,
                                   const PathOptimizationNS::FgEvalReferenceSmoothing::ADvector &vars) {
    size_t point_num = seg_s_list_.size();
    size_t x_idx_begin = 0;
    size_t y_idx_begin = x_idx_begin + point_num;
    size_t theta_idx_begin = y_idx_begin + point_num;
    size_t k_idx_begin = theta_idx_begin + point_num;
    size_t cons_x_idx_begin = 1;
    size_t cons_y_idx_begin = cons_x_idx_begin + point_num - 1;
    size_t cons_theta_idx_begin = cons_y_idx_begin + point_num - 1;
    for (size_t i = 0; i != point_num - 1; ++i) {
        ad cur_x = vars[x_idx_begin + i];
        ad next_x = vars[x_idx_begin + i + 1];
        ad ref_x = seg_x_list_[i];
        ad cur_y = vars[y_idx_begin + i];
        ad next_y = vars[y_idx_begin + i + 1];
        ad ref_y = seg_y_list_[i];
        ad cur_theta = vars[theta_idx_begin + i];
        ad next_theta = vars[theta_idx_begin + i + 1];
        ad ref_theta = seg_angle_list_[i];
        ad ds = seg_s_list_[i + 1] - seg_s_list_[i];
        ad cur_k = vars[k_idx_begin + i];

        // cost
        fg[0] += 0.001 * (pow(cur_x - ref_x, 2) + pow(cur_y - ref_y, 2));
        fg[0] += 1 * pow(cur_k, 2);
        if (i != 0) {
            ad pre_k = vars[k_idx_begin + i - 1];
            fg[0] += 10 * pow(cur_k - pre_k, 2);
        }
        // cons
        fg[cons_x_idx_begin + i] = next_x - (cur_x + ds * (cos(ref_theta) - sin(ref_theta) * cur_theta + ref_theta * sin(ref_theta)));
        fg[cons_y_idx_begin + i] = next_y - (cur_y + ds * (sin(ref_theta) + cos(ref_theta) * cur_theta - ref_theta * cos(ref_theta)));
        fg[cons_theta_idx_begin + i] = next_theta - (cur_theta + ds * cur_k);
    }
}

QPSmoother::QPSmoother(const std::vector<PathOptimizationNS::State> &input_points,
                       const PathOptimizationNS::State &start_state,
                       const PathOptimizationNS::Map &grid_map) :
    TensionSmoother(input_points, start_state, grid_map) {}

bool QPSmoother::ipoptSmooth(const std::vector<double> &x_list,
                             const std::vector<double> &y_list,
                             const std::vector<double> &angle_list,
                             const std::vector<double> &s_list,
                             std::vector<double> *result_x_list,
                             std::vector<double> *result_y_list,
                             std::vector<double> *result_s_list) {
    LOG(INFO) << "QP smoother test: ipopt";
    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), angle_list.size());
    CHECK_EQ(angle_list.size(), s_list.size());
    typedef CPPAD_TESTVECTOR(double) Dvector;
    auto point_num = x_list.size();
    size_t n_vars = 3 * point_num + point_num - 1;
    Dvector vars(n_vars);
    size_t x_idx_begin = 0;
    size_t y_idx_begin = x_idx_begin + point_num;
    size_t theta_idx_begin = y_idx_begin + point_num;
    size_t k_idx_begin = theta_idx_begin + point_num;
    for (size_t i = 0; i < point_num; i++) {
        vars[x_idx_begin + i] = x_list[i];
        vars[y_idx_begin + i] = y_list[i];
        vars[theta_idx_begin + i] = angle_list[i];
        if (i != point_num - 1) vars[k_idx_begin + i] = 0;
    }
    // bounds of variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (size_t i = 0; i < point_num; i++) {
        vars_lowerbound[x_idx_begin + i] = -DBL_MAX;
        vars_upperbound[x_idx_begin + i] = DBL_MAX;
        vars_lowerbound[y_idx_begin + i] = -DBL_MAX;
        vars_upperbound[y_idx_begin + i] = DBL_MAX;
        vars_lowerbound[theta_idx_begin + i] = -M_PI;
        vars_upperbound[theta_idx_begin + i] = M_PI;
        if (i != point_num - 1) {
            vars_lowerbound[k_idx_begin + i] = -tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
            vars_upperbound[k_idx_begin + i] = tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
        }
    }
    vars_lowerbound[x_idx_begin] = vars_upperbound[x_idx_begin] = x_list[0];
    vars_lowerbound[y_idx_begin] = vars_upperbound[y_idx_begin] = y_list[0];
    // Constraints
    size_t n_constraints = (point_num - 1) * 3;
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i != n_constraints; ++i) {
        constraints_lowerbound[i] = constraints_upperbound[i] = 0;
    }
    // options for IPOPT solver
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    FgEvalQPSmoothing fg_eval_reference_smoothing(x_list,
                                                  y_list,
                                                  s_list,
                                                  angle_list);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalQPSmoothing>(options, vars,
                                                    vars_lowerbound, vars_upperbound,
                                                    constraints_lowerbound, constraints_upperbound,
                                                    fg_eval_reference_smoothing, solution);
    // Check if it works
    bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(WARNING) << "Tension smoothing ipopt solver failed!";
        return false;
    }
    // output
    result_s_list->clear();
    result_x_list->clear();
    result_y_list->clear();
    double tmp_s = 0;
    for (size_t i = 0; i != point_num; ++i) {
        result_x_list->emplace_back(solution.x[x_idx_begin + i]);
        result_y_list->emplace_back(solution.x[y_idx_begin + i]);
        if (i != 0)
            tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
                              + pow(result_y_list->at(i) - result_y_list->at(i - 1), 2));
        result_s_list->emplace_back(tmp_s);
    }
    return true;
}
}