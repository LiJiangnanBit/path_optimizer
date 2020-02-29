//
// Created by ljn on 20-2-27.
//

#ifndef TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SOLVER_SOLVER_IPOPT_KPVP_HPP_
#define TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SOLVER_SOLVER_IPOPT_KPVP_HPP_

#include "trajectory_optimizer/solver/solver_ipopt_interface.hpp"

namespace TrajOptNS {

class FgEvalKpvp {
public:
    FgEvalKpvp() = default;
    void init(size_t horizon, const std::shared_ptr<SolverInput> &input);
    typedef AD<double> ADd;
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars);
    size_t horizon_;
    std::shared_ptr<SolverInput> input_;
};

class SolverIpoptKpvp : public SolverIpoptInterface {
public:
    SolverIpoptKpvp() : SolverIpoptInterface() {}
    ~SolverIpoptKpvp() {};
    void init(const std::shared_ptr<SolverInput> &input) override;
    bool solve(std::vector<State> *result_trajectory) override;

private:
    FgEvalKpvp fg_eval_kpvp_;
};
}
#endif //TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SOLVER_SOLVER_IPOPT_KPVP_HPP_
