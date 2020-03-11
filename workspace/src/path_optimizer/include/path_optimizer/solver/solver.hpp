//
// Created by ljn on 20-3-10.
//

#ifndef PATH_OPTIMIZER_SOLVER_HPP
#define PATH_OPTIMIZER_SOLVER_HPP

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

namespace PathOptimizationNS {

class Config;
class ReferencePath;
class VehicleState;
class State;

class OsqpSolver {

 public:
  OsqpSolver() = delete;

  OsqpSolver(const Config &config,
             const ReferencePath &reference_path,
             const VehicleState &vehicle_state,
             const size_t &horizon) :
      config_(config),
      horizon_(horizon),
      reference_path_(reference_path),
      vehicle_state_(vehicle_state) {}

  ~OsqpSolver() = default;

  virtual bool solve(std::vector<State> *optimized_path) = 0;

 protected:
  // Set Matrices for osqp solver.
  virtual void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const = 0;

  virtual void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                   Eigen::VectorXd *lower_bound,
                                   Eigen::VectorXd *upper_bound) const = 0;

  const Config &config_;
  const size_t horizon_{};
  const ReferencePath &reference_path_;
  const VehicleState &vehicle_state_;
  OsqpEigen::Solver solver_;

};

}
#endif //PATH_OPTIMIZER_SOLVER_HPP
