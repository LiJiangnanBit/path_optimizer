//
// Created by ljn on 20-1-26.
//
#include <path_optimizer/path_optimizer.hpp>
#include "reference_path_smoother/frenet_reference_path_smoother.hpp"

namespace PathOptimizationNS {

FrenetReferencePathSmoother::FrenetReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                                                         const hmpl::State &start_state,
                                                         const hmpl::InternalGridMap &grid_map,
                                                         const Config &config) :
    points_list_(input_points),
    start_state_(start_state),
    grid_map_(grid_map),
    config_(config) {}

bool FrenetReferencePathSmoother::smooth(ReferencePath *reference_path,
                                         std::vector<hmpl::State> *smoothed_path_display) const {
    return smoothPathFrenet(&reference_path->x_s_,
                            &reference_path->y_s_,
                            &reference_path->max_s_,
                            smoothed_path_display);
}

bool FrenetReferencePathSmoother::smoothPathFrenet(tk::spline *x_s_out,
                                                   tk::spline *y_s_out,
                                                   double *max_s_out,
                                                   std::vector<hmpl::State> *smoothed_path_display) const {
    auto sm_start = std::clock();
    std::vector<double> x_list, y_list, s_list, angle_list;
    // B spline smoothing.
    double length = 0;
    for (size_t i = 0; i != points_list_.size() - 1; ++i) {
        length += hmpl::distance(points_list_[i], points_list_[i + 1]);
    }
    tinyspline::BSpline b_spline_raw(points_list_.size(), 2, 5);
    std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
    for (size_t i = 0; i != points_list_.size(); ++i) {
        ctrlp_raw[2 * (i)] = points_list_[i].x;
        ctrlp_raw[2 * (i) + 1] = points_list_[i].y;
    }
    b_spline_raw.setControlPoints(ctrlp_raw);
    double delta_t = 1.0 / length;
    double tmp_t = 0;
    while (tmp_t <= 1) {
        auto result = b_spline_raw.eval(tmp_t).result();
        x_list.emplace_back(result[0]);
        y_list.emplace_back(result[1]);
        tmp_t += delta_t;
    }
    auto result = b_spline_raw.eval(1).result();
    x_list.emplace_back(result[0]);
    y_list.emplace_back(result[1]);
    s_list.emplace_back(0);
    for (size_t i = 1; i != x_list.size(); ++i) {
        double dis = sqrt(pow(x_list[i] - x_list[i - 1], 2) + pow(y_list[i] - y_list[i - 1], 2));
        s_list.emplace_back(s_list.back() + dis);
    }
    tk::spline x_spline, y_spline;
    x_spline.set_points(s_list, x_list);
    y_spline.set_points(s_list, y_list);
    double max_s = s_list.back();
    // Make the path dense, the interval being 0.3m
    x_list.clear();
    y_list.clear();
    s_list.clear();
    // Divid the reference path.
    double delta_beginning_s = 4;
    double delta_s = 2;
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

    auto sm_pre_solve = std::clock();
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
    // TODO: use a config file
    std::vector<double> weights;
    weights.push_back(20); //curvature weight
    weights.push_back(30); //curvature rate weight
    weights.push_back(0.01); //distance to boundary weight
    weights.push_back(1); //path length weight
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
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    auto sm_solved = std::clock();
    if (!ok) {
        LOG(WARNING) << "smoothing solver failed!";
        return false;
    }
    LOG(INFO) << "smoothing solver succeeded!";
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
    // Output.
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
            hmpl::State state;
            state.x = x;
            state.y = y;
            smoothed_path_display->push_back(state);
        }
        tmp_s_2 += 0.3;
    }
    x_s_out->set_points(result_s_list, result_x_list);
    y_s_out->set_points(result_s_list, result_y_list);
    *max_s_out = result_s_list.back();
    auto sm_end = std::clock();
    printf("*********\n"
           "sm_pre: %f\n sm_solve: %f\n sm_after: %f\n sm_all: %f\n"
           "**********\n",
           (double) (sm_pre_solve - sm_start) / CLOCKS_PER_SEC,
           (double) (sm_solved - sm_pre_solve) / CLOCKS_PER_SEC,
           (double) (sm_end - sm_solved) / CLOCKS_PER_SEC,
           (double) (sm_end - sm_start) / CLOCKS_PER_SEC);
    return true;
}

}
