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
        horizon_(0),
        initialized_(false)
        {};
    virtual void init(const std::shared_ptr<SolverInput> &input) = 0;
    virtual bool solve(std::vector<State> *result_trajectory) = 0;
    virtual ~Solver() {};
protected:
    size_t horizon_{};
    bool initialized_{};
};
}
#endif //TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SLOVER_SOLVER_HPP_
