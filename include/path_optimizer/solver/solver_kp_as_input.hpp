//
// Created by ljn on 20-3-10.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_KP_AS_INPUT_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_KP_AS_INPUT_HPP_

#include "solver.hpp"

namespace PathOptimizationNS {

class SolverKpAsInput : public OsqpSolver {
 public:
    SolverKpAsInput() = delete;

    SolverKpAsInput(const ReferencePath &reference_path,
                    const VehicleState &vehicle_state,
                    const size_t &horizon);

    ~SolverKpAsInput() override = default;

//  bool solve(std::vector<State> *optimized_path) override ;

 private:

    void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const override;

    void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const override;
    void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                          std::vector<PathOptimizationNS::State> *optimized_path) const override;
    const int keep_control_steps_{};
    const size_t control_horizon_{};
    const size_t state_size_{};
    const size_t control_size_{};
    const size_t slack_size_{};
};
}

#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_KP_AS_INPUT_HPP_
