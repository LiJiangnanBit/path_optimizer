#ifndef SOLVER_OSQP_HPP
#define SOLVER_OSQP_HPP

#include "trajectory_optimizer/solver/solver.hpp"

namespace TrajOptNS {
class SolverOsqpInterface : public Solver {
public:
    SolverOsqpInterface() : Solver() {};
    ~SolverOsqpInterface() {};
protected:
    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    Eigen::SparseMatrix<double> linear_matrix_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;
    OsqpEigen::Solver solver_;   
};

}
#endif