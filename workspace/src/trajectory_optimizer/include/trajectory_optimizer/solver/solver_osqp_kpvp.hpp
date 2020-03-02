#ifndef SOLVER_OSQP_KPVP
#define SOLVER_OSQP_KPVP

#include "trajectory_optimizer/solver/solver_osqp_interface.hpp"

namespace TrajOptNS {

class SolverOsqpKpvp : public SolverOsqpInterface {
public:
    SolverOsqpKpvp() : SolverOsqpInterface() {}
    ~SolverOsqpKpvp() {};
    void init(const std::shared_ptr<SolverInput> &input) override;
    bool solve(std::vector<State> *result_trajectory) override;

protected:
    void setHessianAndGradient() override ;
    void setConstraintMatrix() override ;
};
}
#endif