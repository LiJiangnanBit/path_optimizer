//
// Created by ljn on 20-3-10.
//

#include "path_optimizer/solver/solver.hpp"
#include "path_optimizer/solver/solver_k_as_input.hpp"
#include "path_optimizer/solver/solver_kp_as_input.hpp"
#include "path_optimizer/solver/solver_kp_as_input_constrained.hpp"

namespace PathOptimizationNS {

std::unique_ptr<OsqpSolver> OsqpSolver::create(std::string &type,
                                               const PathOptimizationNS::ReferencePath &reference_path,
                                               const PathOptimizationNS::VehicleState &vehicle_state,
                                               const size_t &horizon) {
    if (type == "K") {
        return std::unique_ptr<OsqpSolver>(new SolverKAsInput(reference_path, vehicle_state, horizon));
    } else if (type == "KP") {
        return std::unique_ptr<OsqpSolver>(new SolverKpAsInput(reference_path, vehicle_state, horizon));
    } else if (type == "KPC") {
        return std::unique_ptr<OsqpSolver>(new SolverKpAsInputConstrained(reference_path, vehicle_state, horizon));
    } else {
        LOG(ERROR) << "No such solver!";
        return nullptr;
    }
}

}