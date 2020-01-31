//
// Created by ljn on 20-1-31.
//

#include "reference_path_smoother/cartesian_reference_path_smoother.hpp"

namespace PathOptimizationNS {

CartesianReferencePathSmoother::CartesianReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                                                               const hmpl::State &start_state,
                                                               const hmpl::InternalGridMap &grid_map,
                                                               const Config &config) :
    points_list_(input_points),
    start_state_(start_state),
    grid_map_(grid_map),
    config_(config) {}

bool CartesianReferencePathSmoother::smooth(ReferencePath *reference_path,
                                            std::vector<hmpl::State> *smoothed_path_display) const {
    return smoothPathCartesian(&reference_path->x_s_,
                               &reference_path->y_s_,
                               &reference_path->max_s_,
                               smoothed_path_display);
}

bool CartesianReferencePathSmoother::smoothPathCartesian(tk::spline *x_s_out,
                                                         tk::spline *y_s_out,
                                                         double *max_s_out,
                                                         std::vector<hmpl::State> *smoothed_path_display) const {
    auto sm_start = std::clock();
    std::vector<double> x_list, y_list, s_list, angle_list;
    tk::spline x_spline, y_spline;
    double s = 0;
    for (size_t i = 0; i != points_list_.size(); ++i) {
        if (i == 0) {
            s_list.emplace_back(0);
        } else {
            double ds = hmpl::distance(points_list_[i], points_list_[i - 1]);
            s += ds;
            s_list.emplace_back(s);
        }
        x_list.emplace_back(points_list_[i].x);
        y_list.emplace_back(points_list_[i].y);
    }
    double max_s = s_list.back();
    std::cout << "ref path length: " << max_s << std::endl;
    x_spline.set_points(s_list, x_list);
    y_spline.set_points(s_list, y_list);
    // Make the path dense, the interval being 0.3m
    x_list.clear();
    y_list.clear();
    s_list.clear();
    // Divide the reference path.
    double delta_s = 0.5;
    s_list.emplace_back(0);
    while (s_list.back() < max_s) {
        s_list.emplace_back(s_list.back() + delta_s);
    }
    if (max_s - s_list.back() > 1) {
        s_list.emplace_back(max_s);
    }
    auto N = s_list.size();
    // Store reference states in vectors. They will be used later.
    for (size_t i = 0; i != N; ++i) {
        double length_on_ref_path = s_list[i];
        double angle = atan2(y_spline.deriv(1, length_on_ref_path), x_spline.deriv(1, length_on_ref_path));
        angle_list.emplace_back(angle);
        x_list.emplace_back(x_spline(length_on_ref_path));
        y_list.emplace_back(y_spline(length_on_ref_path));
    }
    auto sm_pre_solve = std::clock();
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t n_vars = 2 * N;
    const auto x_range_begin = 0;
    const auto y_range_begin = x_range_begin + N;
    Dvector vars(n_vars);
    for (size_t i = 0; i < N; i++) {
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
    vars_lowerbound[x_range_begin] = start_state_.x;
    vars_upperbound[x_range_begin] = start_state_.x;
    vars_lowerbound[y_range_begin] = start_state_.y;
    vars_upperbound[y_range_begin] = start_state_.y;
    // Constraint the last point.
    vars_lowerbound[x_range_begin + N - 1] = x_list.back();
    vars_upperbound[x_range_begin + N - 1] = x_list.back();
    vars_lowerbound[y_range_begin + N - 1] = y_list.back();
    vars_upperbound[y_range_begin + N - 1] = y_list.back();
    // Set constraints.
    // Note that the constraint number should be N - 2.
    size_t n_constraints = N - 1;
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    // Get clearance for each point:
    for (size_t i = 0; i != n_constraints; ++i) {
        double x = x_list[i + 1];
        double y = y_list[i + 1];
        double clearance = grid_map_.getObstacleDistance(grid_map::Position(x, y));
        // Adjust clearance according to the original clearance.
        if (clearance > 1.5) {
            clearance -= 1.5;
        } else {
            clearance *= 0.66;
        }
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = pow(clearance, 2);
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
    options += "Numeric max_cpu_time          0.3\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    // TODO: use a config file
    FgEvalReferenceSmoothing fg_eval_reference_smoothing(x_list,
                                                         y_list,
                                                         s_list,
                                                         N);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalReferenceSmoothing>(options, vars,
                                                           vars_lowerbound, vars_upperbound,
                                                           constraints_lowerbound, constraints_upperbound,
                                                           fg_eval_reference_smoothing, solution);
    // Check if it works
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    auto sm_solved = std::clock();
    if (!ok) {
        LOG(WARNING) << "smoothing solver failed!";
        return false;
    }
    LOG(INFO) << "smoothing solver succeeded!";
    // output
    size_t control_points_num = N;
    tinyspline::BSpline b_spline(control_points_num, 2, 3);
    std::vector<tinyspline::real> ctrlp = b_spline.controlPoints();
    for (size_t i = 0; i != N; ++i) {
        double tmp_x = solution.x[x_range_begin + i];
        double tmp_y = solution.x[y_range_begin + i];
        if (std::isnan(tmp_x) || std::isnan(tmp_y)) {
            LOG(WARNING) << "output is not a number, smoothing failed!" << std::endl;
            return false;
        }
        ctrlp[2 * (i)] = tmp_x;
        ctrlp[2 * (i) + 1] = tmp_y;
    }
    // B spline
    b_spline.setControlPoints(ctrlp);
    auto min_dis_to_vehicle = DBL_MAX;
    size_t min_index_to_vehicle = 0;
    double step_t = 1.0 / (3.0 * N);
    for (size_t i = 0; i <= 3 * N; ++i) {
        double t = i * step_t;
        auto p = b_spline.eval(t).result();
        double tmp_dis = sqrt(pow(p[0] - start_state_.x, 2) + pow(p[1] - start_state_.y, 2));
        if (tmp_dis <= min_dis_to_vehicle) {
            min_dis_to_vehicle = tmp_dis;
            min_index_to_vehicle = i;
        } else if (tmp_dis > 15 && min_dis_to_vehicle < 15) {
            break;
        }
    }
    std::vector<tinyspline::real> result;
    std::vector<tinyspline::real> result_next;
    std::vector<double> x_set, y_set, s_set;
    result = b_spline.eval(min_index_to_vehicle * step_t).result();
    x_set.emplace_back(result[0]);
    y_set.emplace_back(result[1]);
    s_set.emplace_back(0);
    hmpl::State state;
    for (size_t i = min_index_to_vehicle + 1; i <= 3 * N; ++i) {
        double t = i * step_t;
        result = b_spline.eval(t).result();
        double ds = sqrt(pow(result[0] - x_set.back(), 2) + pow(result[1] - y_set.back(), 2));
        x_set.emplace_back(result[0]);
        y_set.emplace_back(result[1]);
        s_set.emplace_back(s_set.back() + ds);
        state.x = result[0];
        state.y = result[1];
    }
    x_s_out->set_points(s_set, x_set);
    y_s_out->set_points(s_set, y_set);
    // Display smoothed reference path.
    if (smoothed_path_display) {
        smoothed_path_display->clear();
        double display_smooth_s = 0;
        while (display_smooth_s < s_set.back()) {
            hmpl::State display_smooth_state;
            display_smooth_state.x = (*x_s_out)(display_smooth_s);
            display_smooth_state.y = (*y_s_out)(display_smooth_s);
            smoothed_path_display->emplace_back(display_smooth_state);
            display_smooth_s += 0.3;
        }
    }
    *max_s_out = s_set.back();
    auto sm_end = std::clock();
    printf("*********\n"
           "reference path smoothing: %f\n solve: %f\n after solving: %f\n all: %f\n"
           "**********\n",
           (double) (sm_pre_solve - sm_start) / CLOCKS_PER_SEC,
           (double) (sm_solved - sm_pre_solve) / CLOCKS_PER_SEC,
           (double) (sm_end - sm_solved) / CLOCKS_PER_SEC,
           (double) (sm_end - sm_start) / CLOCKS_PER_SEC);
    return true;
}
}