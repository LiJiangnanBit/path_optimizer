//
// Created by ljn on 20-2-21.
//

#ifndef TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SLOVER_SOLVER_HPP_
#define TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SLOVER_SOLVER_HPP_

#include <trajectory_optimizer/data_struct/data_struct.hpp>
#include <memory>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

namespace TrajOptNS {

class Solver {
public:
    Solver();
    virtual void init(const SolverInput &input) = 0;
    virtual bool solve() = 0;
protected:
    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    Eigen::SparseMatrix<double> linearMatrix_;
    Eigen::VectorXd lowerBound_;
    Eigen::VectorXd upperBound_;
    int horizon_{};
    bool initialized_{};
    OsqpEigen::Solver solver_;
};
}
#endif //TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SLOVER_SOLVER_HPP_
