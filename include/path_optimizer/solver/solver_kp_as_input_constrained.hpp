//
// Created by ljn on 20-3-10.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_KP_AS_INPUT_CONSTRAIND_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_KP_AS_INPUT_CONSTRAIND_HPP_

#include "solver.hpp"

namespace PathOptimizationNS {

class SolverKpAsInputConstrained : public OsqpSolver {
 public:
  SolverKpAsInputConstrained() = delete;

  SolverKpAsInputConstrained(const Config &config,
                             const ReferencePath &reference_path,
                             const VehicleState &vehicle_state,
                             const size_t &horizon);

  ~SolverKpAsInputConstrained() override = default;

  bool solve(std::vector<State> *optimized_path) override;

 protected:

  void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const override;

  void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                           Eigen::VectorXd *lower_bound,
                           Eigen::VectorXd *upper_bound) const override;

 private:
  const int keep_control_steps_{};
  const size_t control_horizon_{};
  const size_t state_size_{};
  const size_t control_size_{};
  const size_t slack_size_{};
};
}

#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_KP_AS_INPUT_HPP_
