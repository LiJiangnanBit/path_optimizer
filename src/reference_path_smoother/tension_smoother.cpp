//
// Created by ljn on 20-4-14.
//
#include <tinyspline_ros/tinysplinecpp.h>
#include "OsqpEigen/OsqpEigen.h"
#include "glog/logging.h"
#include "path_optimizer/tools/Map.hpp"
#include "path_optimizer/reference_path_smoother/tension_smoother.hpp"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/config/planning_flags.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"

namespace PathOptimizationNS {

void FgEvalReferenceSmoothing::operator()(PathOptimizationNS::FgEvalReferenceSmoothing::ADvector &fg,
                                          const PathOptimizationNS::FgEvalReferenceSmoothing::ADvector &vars) {
    size_t point_num = seg_s_list_.size();
    for (size_t i = 1; i != point_num - 1; ++i) {
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
    }
}

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
    std::vector<double> result_x_list, result_y_list, result_s_list;
    bool solver_ok{false};
    if (FLAGS_tension_solver == "IPOPT") {
        solver_ok = ipoptSmooth(x_list, y_list, angle_list, s_list, &result_x_list, &result_y_list, &result_s_list);
    } else if (FLAGS_tension_solver == "OSQP") {
        solver_ok = osqpSmooth(x_list, y_list, angle_list, s_list, &result_x_list, &result_y_list, &result_s_list);
    } else {
        LOG(ERROR) << "No such solver for tension smoother!";
        return false;
    }
    if (!solver_ok) {
        LOG(ERROR) << "Tension smoother failed!";
        return false;
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
    LOG(INFO) << "Tension smoother succeeded!";
    if (smoothed_path_display) {
        smoothed_path_display->clear();
        for (int i = 0; i != result_x_list.size(); ++i) {
            smoothed_path_display->emplace_back(result_x_list[i], result_y_list[i]);
        }
    }
    return true;
}

bool TensionSmoother::ipoptSmooth(const std::vector<double> &x_list,
                                  const std::vector<double> &y_list,
                                  const std::vector<double> &angle_list,
                                  const std::vector<double> &s_list,
                                  std::vector<double> *result_x_list,
                                  std::vector<double> *result_y_list,
                                  std::vector<double> *result_s_list) {
    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), angle_list.size());
    CHECK_EQ(angle_list.size(), s_list.size());
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t n_vars = x_list.size();
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    // bounds of variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Start point is the start position of the vehicle.
    vars_lowerbound[0] = 0;
    vars_upperbound[0] = 0;
    // Constraint the last point.
    vars_lowerbound[n_vars - 1] = -0.5;
    vars_upperbound[n_vars - 1] = 0.5;
    // Get clearance for each point:
    const double default_clearance{2};
    for (size_t i = 1; i != n_vars - 1; ++i) {
        double x = x_list[i];
        double y = y_list[i];
        double clearance = grid_map_.getObstacleDistance(grid_map::Position(x, y));
        // Adjust clearance.
        if (isEqual(clearance, 0)) {
            clearance = default_clearance;
        } else if (clearance > FLAGS_circle_radius) {
            clearance -= FLAGS_circle_radius;
        }
        vars_lowerbound[i] = -clearance;
        vars_upperbound[i] = clearance;
    }
    // Set constraints.
    // Note that the constraint number should be point_num - 2
    // because the first and the last point is fixed.
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
    options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    FgEvalReferenceSmoothing fg_eval_reference_smoothing(x_list,
                                                         y_list,
                                                         s_list,
                                                         angle_list);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalReferenceSmoothing>(options, vars,
                                                           vars_lowerbound, vars_upperbound,
                                                           constraints_lowerbound, constraints_upperbound,
                                                           fg_eval_reference_smoothing, solution);
    // Check if it works
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(WARNING) << "Tension smoothing ipopt solver failed!";
        return false;
    }
    // output
    result_s_list->clear();
    result_x_list->clear();
    result_y_list->clear();
    double tmp_s = 0;
    for (size_t i = 0; i != n_vars; ++i) {
        double new_angle = constraintAngle(angle_list[i] + M_PI_2);
        double tmp_x = x_list[i] + solution.x[i] * cos(new_angle);
        double tmp_y = y_list[i] + solution.x[i] * sin(new_angle);
        result_x_list->emplace_back(tmp_x);
        result_y_list->emplace_back(tmp_y);
        if (i != 0) tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
                                      + pow(result_y_list->at(i) - result_y_list->at(i - 1), 2));
        result_s_list->emplace_back(tmp_s);
    }
    return true;
}

bool TensionSmoother::osqpSmooth(const std::vector<double> &x_list,
                                 const std::vector<double> &y_list,
                                 const std::vector<double> &angle_list,
                                 const std::vector<double> &s_list,
                                 std::vector<double> *result_x_list,
                                 std::vector<double> *result_y_list,
                                 std::vector<double> *result_s_list) {
    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), angle_list.size());
    CHECK_EQ(angle_list.size(), s_list.size());
    auto point_num = x_list.size();
    OsqpEigen::Solver solver_;
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(3 * point_num);
    solver_.data()->setNumberOfConstraints(3 * point_num);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * point_num);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    setHessianMatrix(point_num, &hessian);
    setConstraintMatrix(x_list, y_list, angle_list, s_list, &linearMatrix, &lowerBound, &upperBound);
    // Input to solver.
    if (!solver_.data()->setHessianMatrix(hessian)) return false;
    if (!solver_.data()->setGradient(gradient)) return false;
    if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver_.data()->setLowerBound(lowerBound)) return false;
    if (!solver_.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver_.initSolver()) return false;
    if (!solver_.solve()) return false;
    const auto &QPSolution{solver_.getSolution()};
    // Output.
    result_s_list->clear();
    result_x_list->clear();
    result_y_list->clear();
    double tmp_s = 0;
    for (size_t i = 0; i != point_num; ++i) {
        double tmp_x = QPSolution(i);
        double tmp_y = QPSolution(point_num + i);
        result_x_list->emplace_back(tmp_x);
        result_y_list->emplace_back(tmp_y);
        if (i != 0) tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
                                      + pow(result_y_list->at(i) - result_y_list->at(i - 1), 2));
        result_s_list->emplace_back(tmp_s);
    }
    return true;
}

void TensionSmoother::setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t x_start_index{0};
    const size_t y_start_index{x_start_index + size};
    const size_t d_start_index{y_start_index + size};
    const size_t matrix_size = 3 * size;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    // Curvature part.
    Eigen::Matrix<double, 3, 1> vec{1, -2, 1};
    Eigen::Matrix3d element{vec * vec.transpose() * FLAGS_cartesian_curvature_weight};
    for (int i = 0; i != size - 2; ++i) {
        hessian.block(x_start_index + i, x_start_index + i, 3, 3) += element;
        hessian.block(y_start_index + i, y_start_index + i, 3, 3) += element;
    }
    // Deviation part.
    for (int i = 0; i != size; ++i) {
        hessian(d_start_index + i, d_start_index + i) = FLAGS_cartesian_deviation_weight;
    }
    *matrix_h = hessian.sparseView();
}

void TensionSmoother::setConstraintMatrix(const std::vector<double> &x_list,
                                          const std::vector<double> &y_list,
                                          const std::vector<double> &angle_list,
                                          const std::vector<double> &s_list,
                                          Eigen::SparseMatrix<double> *matrix_constraints,
                                          Eigen::VectorXd *lower_bound,
                                          Eigen::VectorXd *upper_bound) const {
    const size_t size{x_list.size()};
    const size_t x_start_index{0};
    const size_t y_start_index{x_start_index + size};
    const size_t d_start_index{y_start_index + size};
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * size, 3 * size);
    *lower_bound = Eigen::MatrixXd::Zero(3 * size, 1);
    *upper_bound = Eigen::MatrixXd::Zero(3 * size, 1);
    for (int i = 0; i != size; ++i) {
        // x, y and d
        cons(x_start_index + i, x_start_index + i) = cons(y_start_index + i, y_start_index + i) = 1;
        double theta{angle_list[i] + M_PI_2};
        cons(x_start_index + i, d_start_index + i) = -cos(theta);
        cons(y_start_index + i, d_start_index + i) = -sin(theta);
        // d
        cons(d_start_index + i, d_start_index + i) = 1;
        // bounds
        (*lower_bound)(x_start_index + i) = x_list[i];
        (*upper_bound)(x_start_index + i) = x_list[i];
        (*lower_bound)(y_start_index + i) = y_list[i];
        (*upper_bound)(y_start_index + i) = y_list[i];
    }
    *matrix_constraints = cons.sparseView();
    // d bounds.
    (*lower_bound)(d_start_index) = 0;
    (*upper_bound)(d_start_index) = 0;
    (*lower_bound)(d_start_index + size - 1) = -0.5;
    (*upper_bound)(d_start_index + size - 1) = 0.5;
    const double default_clearance = 2;
    for (size_t i = 1; i != size - 1; ++i) {
        double x = x_list[i];
        double y = y_list[i];
        double clearance = grid_map_.getObstacleDistance(grid_map::Position(x, y));
        // Adjust clearance.
        if (isEqual(clearance, 0)) {
            clearance = default_clearance;
        } else if (clearance > FLAGS_circle_radius) {
            clearance -= FLAGS_circle_radius;
        }
        (*lower_bound)(d_start_index + i) = -clearance;
        (*upper_bound)(d_start_index + i) = clearance;
    }
}

}
