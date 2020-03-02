//
// Created by ljn on 20-2-21.
//

#ifndef TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SLOVER_SOLVER_HPP_
#define TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SLOVER_SOLVER_HPP_

#include <memory>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include "trajectory_optimizer/data_struct/data_struct.hpp"

namespace TrajOptNS {

class Solver {
public:
    Solver() :
        state_horizon_(0),
        control_horizon_{0},
        initialized_(false)
        {};
    virtual void init(const std::shared_ptr<SolverInput> &input) = 0;
    virtual bool solve(std::vector<State> *result_trajectory) = 0;
    virtual ~Solver() {};
protected:
    size_t state_horizon_{};
    size_t control_horizon_{};
    bool initialized_{};
    std::shared_ptr<SolverInput> solver_input_ptr_;
};
}
#endif //TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SLOVER_SOLVER_HPP_
