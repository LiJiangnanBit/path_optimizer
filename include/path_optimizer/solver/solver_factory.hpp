//
// Created by ljn on 20-3-24.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_FACTORY_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_FACTORY_HPP_
#include <memory>

namespace PathOptimizationNS {

class OsqpSolver;
class Config;
class ReferencePath;
class VehicleState;

class SolverFactory {
 public:
    static std::shared_ptr<OsqpSolver> create(const Config &config,
                                              const ReferencePath &reference_path,
                                              const VehicleState &vehicle_state,
                                              const size_t &horizon);
};

}

#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_SOLVER_SOLVER_FACTORY_HPP_
