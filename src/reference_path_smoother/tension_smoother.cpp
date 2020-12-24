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
        ad pre_offset = vars[i - 1];
        ad current_offset = vars[i];
        ad next_offset = vars[i + 1];
        ad pre_x = seg_x_list_[i - 1] + pre_offset * cos(seg_angle_list_[i - 1] + M_PI_2);
        ad pre_y = seg_y_list_[i - 1] + pre_offset * sin(seg_angle_list_[i - 1] + M_PI_2);
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
            * (pow(next_x + pre_x - 2 * current_x, 2) + pow(next_y + pre_y - 2 * current_y, 2));
        if (i > 1) {
            ad pre_pre_offset = vars[i - 2];
            ad pre_pre_x = seg_x_list_[i - 2] + pre_pre_offset * cos(seg_angle_list_[i - 2] + M_PI_2);
            ad pre_pre_y = seg_y_list_[i - 2] + pre_pre_offset * sin(seg_angle_list_[i - 2] + M_PI_2);
            fg[0] += FLAGS_cartesian_curvature_rate_weight *
                (pow(3 * pre_x - 3 * current_x + next_x - pre_pre_x, 2) +
                    pow(3 * pre_y - 3 * current_y + next_y - pre_pre_y, 2));
        }
    }
}

TensionSmoother::TensionSmoother(const std::vector<PathOptimizationNS::State> &input_points,
                                 const PathOptimizationNS::State &start_state,
                                 const PathOptimizationNS::Map &grid_map) :
    ReferencePathSmoother(input_points, start_state, grid_map) {}

bool TensionSmoother::smooth(PathOptimizationNS::ReferencePath *reference_path) {
    std::vector<double> x_list, y_list, s_list, angle_list, k_list;
    if (!segmentRawReference(&x_list, &y_list, &s_list, &angle_list, &k_list)) return false;
    std::vector<double> result_x_list, result_y_list, result_s_list;
    bool solver_ok{false};
    if (FLAGS_tension_solver == "IPOPT") {
        solver_ok = ipoptSmooth(x_list,
                                y_list,
                                angle_list,
                                k_list,
                                s_list,
                                &result_x_list,
                                &result_y_list,
                                &result_s_list);
    } else if (FLAGS_tension_solver == "OSQP") {
        solver_ok = osqpSmooth(x_list,
                               y_list,
                               angle_list,
                               k_list,
                               s_list,
                               &result_x_list,
                               &result_y_list,
                               &result_s_list);
    } else {
        LOG(ERROR) << "No such solver for tension smoother!";
        return false;
    }
    if (!solver_ok) {
        LOG(ERROR) << "Tension smoother failed!";
        return false;
    }
    tk::spline x_spline, y_spline;
    x_spline.set_points(result_s_list, result_x_list);
    y_spline.set_points(result_s_list, result_y_list);

    double max_s_result = result_s_list.back() + 3;
    reference_path->setSpline(x_spline, y_spline, max_s_result);

    x_list_ = std::move(result_x_list);
    y_list_ = std::move(result_y_list);
    s_list_ = std::move(result_s_list);
    return true;
}

bool TensionSmoother::ipoptSmooth(const std::vector<double> &x_list,
                                  const std::vector<double> &y_list,
                                  const std::vector<double> &angle_list,
                                  const std::vector<double> &k_list,
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
    static const double default_clearance{1};
    for (size_t i = 1; i != n_vars - 1; ++i) {
        double x = x_list[i];
        double y = y_list[i];
        double clearance = grid_map_.getObstacleDistance(grid_map::Position(x, y));
        // Adjust clearance.
        clearance = std::max(clearance, default_clearance);
//            isEqual(clearance, 0) ? default_clearance :
//                        clearance > 0.5 ? clearance - 0.5 : clearance;
//                    clearance > FLAGS_circle_radius ? clearance - FLAGS_circle_radius : clearance;
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
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
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
    bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(ERROR) << "Tension smoothing ipopt solver failed!";
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
        if (i != 0)
            tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
                              + pow(result_y_list->at(i) - result_y_list->at(i - 1), 2));
        result_s_list->emplace_back(tmp_s);
    }
    return true;
}

bool TensionSmoother::osqpSmooth(const std::vector<double> &x_list,
                                 const std::vector<double> &y_list,
                                 const std::vector<double> &angle_list,
                                 const std::vector<double> &k_list,
                                 const std::vector<double> &s_list,
                                 std::vector<double> *result_x_list,
                                 std::vector<double> *result_y_list,
                                 std::vector<double> *result_s_list) {
    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), angle_list.size());
    CHECK_EQ(angle_list.size(), s_list.size());
    auto point_num = x_list.size();
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(3 * point_num);
    solver.data()->setNumberOfConstraints(3 * point_num);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * point_num);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    setHessianMatrix(point_num, &hessian);
    setConstraintMatrix(x_list, y_list, angle_list, k_list, s_list, &linearMatrix, &lowerBound, &upperBound);
    // Input to solver.
    if (!solver.data()->setHessianMatrix(hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver.initSolver()) return false;
    if (!solver.solve()) return false;
    const auto &QPSolution{solver.getSolution()};
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
        if (i != 0)
            tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
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
    Eigen::Matrix<double, 3, 1> dds_vec{1, -2, 1};
    Eigen::Matrix3d dds_part{dds_vec * dds_vec.transpose() * FLAGS_cartesian_curvature_weight};
    Eigen::Matrix<double, 4, 1> ddds_vec{-1, 3, -3, 1};
    Eigen::Matrix4d ddds_part{ddds_vec * ddds_vec.transpose() * FLAGS_cartesian_curvature_rate_weight};
    for (int i = 0; i != size - 2; ++i) {
        hessian.block(x_start_index + i, x_start_index + i, 3, 3) += dds_part;
        hessian.block(y_start_index + i, y_start_index + i, 3, 3) += dds_part;
        if (i != size - 3) {
            hessian.block(x_start_index + i, x_start_index + i, 4, 4) += ddds_part;
            hessian.block(y_start_index + i, y_start_index + i, 4, 4) += ddds_part;
        }
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
                                          const std::vector<double> &k_list,
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
//    const double shrink_clearance = 0;
    for (size_t i = 1; i != size - 1; ++i) {
        double x = x_list[i];
        double y = y_list[i];
        double clearance = grid_map_.getObstacleDistance(grid_map::Position(x, y));
        // Adjust clearance.
        clearance = std::min(clearance, default_clearance);
        LOG(INFO) << "id: " << i << ", " << clearance;
//            isEqual(clearance, 0) ? default_clearance :
//                   clearance > shrink_clearance ? clearance - shrink_clearance : clearance;
        (*lower_bound)(d_start_index + i) = -clearance;
        (*upper_bound)(d_start_index + i) = clearance;
    }
}

}
