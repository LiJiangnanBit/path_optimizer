//
// Created by ljn on 20-1-27.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_INTERFACE_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_INTERFACE_HPP_
#include <Eigen/Dense>
#include <cassert>
#include <OsqpEigen/OsqpEigen.h>

namespace PathOptimizationNS {

class Config;
class ReferencePath;
class VehicleState;

class SolverInterface {
public:
    SolverInterface() = delete;

    SolverInterface(const Config &config,
                    const ReferencePath &reference_path,
                    const VehicleState &vehicle_state,
                    const size_t &horizon);

    // Core function.
    bool solve(Eigen::VectorXd *solution);

    // Used to sample paths. Call initializeSampling first and then solveSampling.
    bool initializeSampling(double target_angle, double angle_error_allowed, double offset_error_allowed);
    bool solveSampling(Eigen::VectorXd *solution,
                       double offset);

    // For dynamic obstacle avoidance.
    bool solveDynamic(Eigen::VectorXd *solution);
    bool solveDynamicUpdate(Eigen::VectorXd *solution,
                            const std::vector<std::vector<double>> &clearance);
private:
    // Set Matrices for osqp solver.
    void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const;

    void setDynamicMatrix(size_t n,
                          const std::vector<double> &seg_s_list,
                          const std::vector<double> &seg_k_list,
                          Eigen::Matrix<double, 2, 2> *matrix_a,
                          Eigen::Matrix<double, 2, 1> *matrix_b) const;

    void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const;

    void setConstraintMatrixWithOffset(double offset,
                                       double target_angle,
                                       double angle_error_allowed,
                                       double offset_error_allowed,
                                       Eigen::SparseMatrix<double> *matrix_constraints,
                                       Eigen::VectorXd *lower_bound,
                                       Eigen::VectorXd *upper_bound) const;

    const Config &config_;
    const size_t horizon_{};
    const ReferencePath &reference_path_;
    const VehicleState &vehicle_state_;
    // Solvers
    // TODO: use derived classes instead of putting all solvers in one class.
    OsqpEigen::Solver solver_;
    OsqpEigen::Solver solver_for_sampling_;
    OsqpEigen::Solver solver_for_dynamic_env_;
    bool solver_for_sampling_initialized_flag_{false};
    bool solver_for_dynamic_initialized_flag_{false};
    Eigen::VectorXd lowerBound_;
    Eigen::VectorXd upperBound_;
    double offset_error_allowed_{};
};
} // namespace
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_INTERFACE_HPP_
