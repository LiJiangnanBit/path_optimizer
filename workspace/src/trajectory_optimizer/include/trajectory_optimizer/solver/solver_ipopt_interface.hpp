#ifndef SOLVER_IPOPT_INTERFACE_HPP
#define SOLVER_IPOPT_INTERFACE_HPP

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "trajectory_optimizer/solver/solver.hpp"

using CppAD::AD;
using Dvector = CPPAD_TESTVECTOR(double);

namespace TrajOptNS {

class SolverIpoptInterface : public Solver {
public:
    SolverIpoptInterface() : Solver() {};
    ~SolverIpoptInterface() {};
    const CppAD::ipopt::solve_result<Dvector> &getSolution() {
        return solution_;
    }

protected:
    Dvector vars_;
    Dvector vars_lowerbound_, vars_upperbound_;
    Dvector cons_lowerbound_, cons_upperbound_;
    CppAD::ipopt::solve_result<Dvector> solution_;

};
}


#endif