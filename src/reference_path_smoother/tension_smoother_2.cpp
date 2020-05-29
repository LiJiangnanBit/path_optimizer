//
// Created by ljn on 20-5-4.
//
#include <tinyspline_ros/tinysplinecpp.h>
#include "OsqpEigen/OsqpEigen.h"
#include "glog/logging.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "path_optimizer/tools/Map.hpp"
#include "path_optimizer/reference_path_smoother/tension_smoother_2.hpp"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/config/planning_flags.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"

namespace PathOptimizationNS {

FgEvalQPSmoothing::FgEvalQPSmoothing(const std::vector<double> &seg_x_list,
                                     const std::vector<double> &seg_y_list,
                                     const std::vector<double> &seg_s_list,
                                     const std::vector<double> &seg_angle_list,
                                     const std::vector<double> &seg_k_list) :
    FgEvalReferenceSmoothing(seg_x_list, seg_y_list, seg_s_list, seg_angle_list),
    seg_k_list_(seg_k_list) {}

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
        ad ref_k = seg_k_list_[i];

        // cost
        fg[0] += FLAGS_tension_2_deviation_weight * (pow(cur_x - ref_x, 2) + pow(cur_y - ref_y, 2));
        fg[0] += FLAGS_tension_2_curvature_weight * pow(cur_k, 2);
        if (i != 0) {
            ad pre_k = vars[k_idx_begin + i - 1];
            fg[0] += FLAGS_tension_2_curvature_rate_weight * pow(cur_k - pre_k, 2);
        }
        // cons
        fg[cons_x_idx_begin + i] =
            next_x - (cur_x + ds * (cos(ref_theta) - sin(ref_theta) * cur_theta));
        fg[cons_y_idx_begin + i] =
            next_y - (cur_y + ds * (sin(ref_theta) + cos(ref_theta) * cur_theta));
        fg[cons_theta_idx_begin + i] = next_theta - (cur_theta + ds * (cur_k - ref_k));
    }
}

TensionSmoother2::TensionSmoother2(const std::vector<PathOptimizationNS::State> &input_points,
                                   const PathOptimizationNS::State &start_state,
                                   const PathOptimizationNS::Map &grid_map) :
    TensionSmoother(input_points, start_state, grid_map) {}

bool TensionSmoother2::ipoptSmooth(const std::vector<double> &x_list,
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
    auto point_num = x_list.size();
    size_t n_vars = 4 * point_num;
    Dvector vars(n_vars);
    size_t x_idx_begin = 0;
    size_t y_idx_begin = x_idx_begin + point_num;
    size_t theta_idx_begin = y_idx_begin + point_num;
    size_t k_idx_begin = theta_idx_begin + point_num;
    for (size_t i = 0; i < point_num; i++) {
        vars[x_idx_begin + i] = x_list[i];
        vars[y_idx_begin + i] = y_list[i];
        vars[theta_idx_begin + i] = 0;
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
        vars_lowerbound[theta_idx_begin + i] = -DBL_MAX;
        vars_upperbound[theta_idx_begin + i] = DBL_MAX;
        if (i != point_num - 1) {
            vars_lowerbound[k_idx_begin + i] = -0.3;//-tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
            vars_upperbound[k_idx_begin + i] = 0.3; //tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
        }
    }
    vars_lowerbound[x_idx_begin] = vars_upperbound[x_idx_begin] = x_list.front();
    vars_lowerbound[y_idx_begin] = vars_upperbound[y_idx_begin] = y_list.front();
    // Constraints.
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
    options += "Numeric max_cpu_time          1\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    FgEvalQPSmoothing fg_eval_reference_smoothing(x_list,
                                                  y_list,
                                                  s_list,
                                                  angle_list,
                                                  k_list);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalQPSmoothing>(options, vars,
                                                    vars_lowerbound, vars_upperbound,
                                                    constraints_lowerbound, constraints_upperbound,
                                                    fg_eval_reference_smoothing, solution);
    // Check if it works
    bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(ERROR) << "Tension smoothing 2 ipopt solver failed!";
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
    LOG(INFO) << "Tension smoothing 2 ipopt solver succeeded!";
    return true;
}

bool TensionSmoother2::osqpSmooth(const std::vector<double> &x_list,
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
    solver.data()->setNumberOfVariables(4 * point_num - 1);
    solver.data()->setNumberOfConstraints(3 * (point_num - 1) + 2);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    setHessianMatrix(point_num, &hessian);
    setGradient(x_list, y_list, &gradient);
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

void TensionSmoother2::setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    const size_t theta_start_index = y_start_index + size;
    const size_t k_start_index = theta_start_index + size;
    const size_t matrix_size = 4 * size - 1;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    // Deviation and curvature.
    for (int i = 0; i != size; ++i) {
        hessian(x_start_index + i, x_start_index + i) = hessian(y_start_index + i, y_start_index + i)
            = FLAGS_tension_2_deviation_weight * 2;
        if (i != size - 1) hessian(k_start_index + i, k_start_index + i) = FLAGS_tension_2_curvature_weight * 2;
    }
    // Curvature change.
    Eigen::Vector2d coeff_vec{1, -1};
    Eigen::Matrix2d coeff = coeff_vec * coeff_vec.transpose();
    for (int i = 0; i != size - 2; ++i) {
        hessian.block(k_start_index + i, k_start_index + i, 2, 2) += 2 * FLAGS_tension_2_curvature_rate_weight * coeff;
    }
    *matrix_h = hessian.sparseView();
}

void TensionSmoother2::setConstraintMatrix(const std::vector<double> &x_list,
                                           const std::vector<double> &y_list,
                                           const std::vector<double> &angle_list,
                                           const std::vector<double> &k_list,
                                           const std::vector<double> &s_list,
                                           Eigen::SparseMatrix<double> *matrix_constraints,
                                           Eigen::VectorXd *lower_bound,
                                           Eigen::VectorXd *upper_bound) const {
    const size_t size = x_list.size();
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    const size_t theta_start_index = y_start_index + size;
    const size_t k_start_index = theta_start_index + size;
    const size_t cons_x_update_start_index = 0;
    const size_t cons_y_update_start_index = cons_x_update_start_index + size - 1;
    const size_t cons_theta_update_start_index = cons_y_update_start_index + size - 1;
    const size_t cons_x_index = cons_theta_update_start_index + size - 1;
    const size_t cons_y_index = cons_x_index + 1;

    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 4 * size - 1);
    *lower_bound = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 1);
    *upper_bound = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 1);
    // Cons.
    for (int i = 0; i != size - 1; ++i) {
        const double ds = s_list[i + 1] - s_list[i];
        cons(cons_x_update_start_index + i, x_start_index + i + 1) =
        cons(cons_y_update_start_index + i, y_start_index + i + 1)
            = cons(cons_theta_update_start_index + i, theta_start_index + i + 1) = 1;
        cons(cons_x_update_start_index + i, x_start_index + i) = cons(cons_y_update_start_index + i, y_start_index + i)
            = cons(cons_theta_update_start_index + i, theta_start_index + i) = -1;
        cons(cons_x_update_start_index + i, theta_start_index + i) = ds * sin(angle_list[i]);
        cons(cons_y_update_start_index + i, theta_start_index + i) = -ds * cos(angle_list[i]);
        cons(cons_theta_update_start_index + i, k_start_index + i) = -ds;
    }
    cons(cons_x_index, x_start_index) = cons(cons_y_index, y_start_index) = 1;
    *matrix_constraints = cons.sparseView();
    // Bounds.
    for (int i = 0; i != size - 1; ++i) {
        const double ds = s_list[i + 1] - s_list[i];
        (*lower_bound)(cons_x_update_start_index + i) = (*upper_bound)(cons_x_update_start_index + i) =
            ds * cos(angle_list[i]);
        (*lower_bound)(cons_y_update_start_index + i) = (*upper_bound)(cons_y_update_start_index + i) =
            ds * sin(angle_list[i]);
        (*lower_bound)(cons_theta_update_start_index + i) = (*upper_bound)(cons_theta_update_start_index + i) =
            -ds * k_list[i];

    }
    (*lower_bound)(cons_x_index) = (*upper_bound)(cons_x_index) = x_list[0];
    (*lower_bound)(cons_y_index) = (*upper_bound)(cons_y_index) = y_list[0];
}

void TensionSmoother2::setGradient(const std::vector<double> &x_list,
                                   const std::vector<double> &y_list,
                                   Eigen::VectorXd *gradient) {
    const auto size = x_list.size();
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    *gradient = Eigen::VectorXd::Constant(4 * size - 1, 0);
    for (int i = 0; i != size; ++i) {
        (*gradient)(x_start_index + i) = -2 * FLAGS_tension_2_deviation_weight * x_list[i];
        (*gradient)(y_start_index + i) = -2 * FLAGS_tension_2_deviation_weight * y_list[i];
    }
}
}