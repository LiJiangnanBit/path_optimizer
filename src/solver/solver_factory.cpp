//
// Created by ljn on 20-3-24.
//
#include "path_optimizer/solver/solver_factory.hpp"
#include "path_optimizer/solver/solver_k_as_input.hpp"
#include "path_optimizer/solver/solver_kp_as_input.hpp"
#include "path_optimizer/solver/solver_kp_as_input_constrained.hpp"
#include "glog/logging.h"
#include "path_optimizer/config/planning_flags.hpp"

namespace PathOptimizationNS {
std::shared_ptr<OsqpSolver> SolverFactory::create(const PathOptimizationNS::ReferencePath &reference_path,
                                                  const PathOptimizationNS::VehicleState &vehicle_state,
                                                  const size_t &horizon) {
    if (FLAGS_optimization_method == "K") {
        LOG(INFO) << "Creating solver type " << "K.";
        return std::make_shared<SolverKAsInput>(reference_path, vehicle_state, horizon);
    } else if (FLAGS_optimization_method == "KP") {
        LOG(INFO) << "Creating solver type " << "KP.";
        return std::make_shared<SolverKpAsInput>(reference_path, vehicle_state, horizon);
    } else if (FLAGS_optimization_method == "KPC") {
        LOG(INFO) << "Creating solver type " << "KPC.";
        return std::make_shared<SolverKpAsInputConstrained>(reference_path, vehicle_state, horizon);
    } else {
        LOG(ERROR) << "No such solver!";
        return nullptr;
    }
}
}
