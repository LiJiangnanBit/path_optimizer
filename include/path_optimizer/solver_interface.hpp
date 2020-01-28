//
// Created by ljn on 20-1-27.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_INTERFACE_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_INTERFACE_HPP_
#include <opt_utils/opt_utils.hpp>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include "data_struct/data_struct.hpp"
#include "config/config.hpp"

namespace PathOptimizationNS {

class SolverInterface {
public:
    SolverInterface();
    SolverInterface(const Config &config,
                    const ReferencePath &reference_path,
                    const VehicleState &vehicle_state,
                    const size_t &horizon);
    bool solve(Eigen::VectorXd *solution);
    bool initializeSampling(double target_angle, double angle_error_allowed, double offset_error_allowed);
    bool solveSampling(Eigen::VectorXd *solution,
                       double offset);
    bool solveDynamic(Eigen::VectorXd *solution);
    bool solveDynamicUpdate(Eigen::VectorXd *solution,
                            const std::vector<std::vector<double>> &clearance);
private:
    void setHessianMatrix(size_t horizon, Eigen::SparseMatrix<double> *matrix_h) const;
    void setDynamicMatrix(size_t n,
                          const std::vector<double> &seg_s_list,
                          const std::vector<double> &seg_k_list,
                          Eigen::Matrix<double, 2, 2> *matrix_a,
                          Eigen::Matrix<double, 2, 1> *matrix_b) const;
    void setConstraintMatrix(size_t horizon,
                             const ReferencePath &reference_path,
                             const VehicleState &vehicle_state,
                             Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const;
    void setConstraintMatrixWithOffset(size_t horizon,
                                       const ReferencePath &reference_path,
                                       const VehicleState &vehicle_state,
                                       double offset,
                                       double target_angle,
                                       double angle_error_allowed,
                                       double offset_error_allowed,
                                       Eigen::SparseMatrix<double> *matrix_constraints,
                                       Eigen::VectorXd *lower_bound,
                                       Eigen::VectorXd *upper_bound) const;

    const Config &config_;
    const size_t &horizon_;
    const ReferencePath &reference_path_;
    const VehicleState &vehicle_state_;
//    bool initialized;
    // Solvers
    OsqpEigen::Solver solver_;
    OsqpEigen::Solver solver_for_sampling_;
    OsqpEigen::Solver solver_for_dynamic_env_;
    bool solver_for_sampling_initialized_flag_;
    bool solver_for_dynamic_initialized_flag_;
    Eigen::VectorXd lowerBound_;
    Eigen::VectorXd upperBound_;
    double offset_error_allowed_;
};
} // namespace
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_INTERFACE_HPP_
