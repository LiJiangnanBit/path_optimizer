//
// Created by ljn on 20-3-10.
//

#ifndef PATH_OPTIMIZER_SOLVER_HPP
#define PATH_OPTIMIZER_SOLVER_HPP

#include <Eigen/Dense>
#include <memory>
#include <OsqpEigen/OsqpEigen.h>
#include "glog/logging.h"

namespace PathOptimizationNS {

class Config;
class ReferencePath;
class VehicleState;
class State;

class OsqpSolver {

 public:
    OsqpSolver() = delete;

    OsqpSolver(const ReferencePath &reference_path,
               const VehicleState &vehicle_state,
               const size_t &horizon);

    virtual ~OsqpSolver() = default;

    static std::unique_ptr<OsqpSolver> create(std::string &type,
                                              const ReferencePath &reference_path,
                                              const VehicleState &vehicle_state,
                                              const size_t &horizon);

    virtual bool solve(std::vector<State> *optimized_path);

 private:
    // Set Matrices for osqp solver.
    virtual void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const = 0;

    virtual void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                     Eigen::VectorXd *lower_bound,
                                     Eigen::VectorXd *upper_bound) const = 0;
    virtual void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                  std::vector<State> *optimized_path) const = 0;

 protected:
    const size_t horizon_{};
    const ReferencePath &reference_path_;
    const VehicleState &vehicle_state_;
    OsqpEigen::Solver solver_;
    double reference_interval_;
    int num_of_variables_, num_of_constraints_;

};

}
#endif //PATH_OPTIMIZER_SOLVER_HPP
