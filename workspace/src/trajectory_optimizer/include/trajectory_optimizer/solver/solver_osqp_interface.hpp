#ifndef SOLVER_OSQP_HPP
#define SOLVER_OSQP_HPP

#include <OsqpEigen/OsqpEigen.h>
#include "trajectory_optimizer/solver/solver.hpp"

namespace TrajOptNS {
class SolverOsqpInterface : public Solver {
public:
    SolverOsqpInterface() : Solver() {};
    ~SolverOsqpInterface() {};

protected:
    virtual void setHessianAndGradient() = 0;
    virtual void setConstraintMatrix() = 0;
    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    Eigen::SparseMatrix<double> linear_matrix_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;
    OsqpEigen::Solver solver_;
    size_t state_vars_num_{}, control_vars_num_{}, slack_vars_num{}, vars_num_{}, cons_num_{};
    size_t state_size_{}, control_size_{};
    Eigen::VectorXd solution_;
};

}
#endif