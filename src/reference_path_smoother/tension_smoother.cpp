//
// Created by ljn on 20-4-14.
//
#include "glog/logging.h"
#include "path_optimizer/tools/Map.hpp"
#include "path_optimizer/reference_path_smoother/tension_smoother.hpp"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"

namespace PathOptimizationNS {

TensionSmoother::TensionSmoother(const std::vector<PathOptimizationNS::State> &input_points,
                                 const PathOptimizationNS::State &start_state,
                                 const PathOptimizationNS::Map &grid_map) :
    ReferencePathSmoother(input_points, start_state, grid_map) {}

bool TensionSmoother::smooth(PathOptimizationNS::ReferencePath *reference_path,
                             std::vector<PathOptimizationNS::State> *smoothed_path_display) {
    CHECK_GT(s_list_.size(), 2);
    CHECK_EQ(x_list_.size(), s_list_.size());
    CHECK_EQ(y_list_.size(), s_list_.size());
    double max_s = s_list_.back();
    std::cout << "ref path length: " << max_s << std::endl;
    tk::spline x_spline, y_spline;
    x_spline.set_points(s_list_, x_list_);
    y_spline.set_points(s_list_, y_list_);
    std::vector<double> x_list, y_list, s_list, angle_list;
    x_list.clear();
    y_list.clear();
    s_list.clear();
    // Divide the reference path.
    double delta_s = 1.0;
    s_list.emplace_back(0);
    while (s_list.back() < max_s) {
        s_list.emplace_back(s_list.back() + delta_s);
    }
    if (max_s - s_list.back() > 1) {
        s_list.emplace_back(max_s);
    }
    auto point_num = s_list.size();
    // Store reference states in vectors. They will be used later.
    for (size_t i = 0; i != point_num; ++i) {
        double length_on_ref_path = s_list[i];
        double angle = atan2(y_spline.deriv(1, length_on_ref_path), x_spline.deriv(1, length_on_ref_path));
        angle_list.emplace_back(angle);
        x_list.emplace_back(x_spline(length_on_ref_path));
        y_list.emplace_back(y_spline(length_on_ref_path));
    }
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t n_vars = 2 * point_num;
    const auto x_range_begin = 0;
    const auto y_range_begin = x_range_begin + point_num;
    Dvector vars(n_vars);
    for (size_t i = 0; i < point_num; i++) {
        vars[i + x_range_begin] = x_list[i];
        vars[i + y_range_begin] = y_list[i];
    }
    // bounds of variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (size_t i = 0; i != n_vars; ++i) {
        vars_lowerbound[i] = -DBL_MAX;
        vars_upperbound[i] = DBL_MAX;
    }
    // Start point is the start position of the vehicle.
    vars_lowerbound[x_range_begin] = x_list.front();
    vars_upperbound[x_range_begin] = x_list.front();
    vars_lowerbound[y_range_begin] = y_list.front();
    vars_upperbound[y_range_begin] = y_list.front();
    // Constraint the last point.
    vars_lowerbound[x_range_begin + point_num - 1] = x_list.back();
    vars_upperbound[x_range_begin + point_num - 1] = x_list.back();
    vars_lowerbound[y_range_begin + point_num - 1] = y_list.back();
    vars_upperbound[y_range_begin + point_num - 1] = y_list.back();
    // Set constraints.
    // Note that the constraint number should be point_num - 2
    // because the first and the last point is fixed.
    size_t n_constraints = point_num - 2;
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    // Get clearance for each point:
    const double default_clearance{2};
    for (size_t i = 1; i != n_constraints - 1; ++i) {
        double x = x_list[i];
        double y = y_list[i];
        double clearance = grid_map_.getObstacleDistance(grid_map::Position(x, y));
        // Adjust clearance.
        if (isEqual(clearance, 0)) {
            clearance = default_clearance;
        } else if (clearance > FLAGS_circle_radius) {
            clearance -= FLAGS_circle_radius;
        }
        constraints_lowerbound[i - 1] = -clearance;
        constraints_upperbound[i - 1] = clearance;
    }
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
    options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    FgEvalReferenceSmoothing fg_eval_reference_smoothing(x_list,
                                                         y_list,
                                                         s_list,
                                                         angle_list,
                                                         point_num);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalReferenceSmoothing>(options, vars,
                                                           vars_lowerbound, vars_upperbound,
                                                           constraints_lowerbound, constraints_upperbound,
                                                           fg_eval_reference_smoothing, solution);
    // Check if it works
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(WARNING) << "Tension smoothing solver failed!";
        return false;
    }
    LOG(INFO) << "Tension smoothing solver succeeded!";
    // output
    std::vector<double> result_x_list, result_y_list, result_s_list;
    double tmp_s = 0;
    for (size_t i = 0; i != point_num; ++i) {
        result_x_list.emplace_back(solution.x[x_range_begin + i]);
        result_y_list.emplace_back(solution.x[y_range_begin + i]);
        if (i != 0) tmp_s += sqrt(pow(result_x_list[i] - result_x_list[i - 1], 2)
                + pow(result_y_list[i] - result_y_list[i - 1], 2));
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
    if (!isEqual(start_distance, 0)) {
        auto min_dis_to_vehicle = start_distance;
        double tmp_s_1 = 0 + 0.1;
        while (tmp_s_1 <= max_s) {
            double x = x_spline(tmp_s_1);
            double y = y_spline(tmp_s_1);
            double dis = sqrt(pow(x - start_state_.x, 2) + pow(y - start_state_.y, 2));
            if (dis <= min_dis_to_vehicle) {
                min_dis_to_vehicle = dis;
                min_dis_s = tmp_s_1;
            } else if (dis > 15 && min_dis_to_vehicle < 15) {
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
