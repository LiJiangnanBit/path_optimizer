//
// Created by ljn on 20-3-10.
//

#include "path_optimizer/solver/solver.hpp"
#include "path_optimizer/solver/solver_k_as_input.hpp"
#include "path_optimizer/solver/solver_kp_as_input.hpp"
#include "path_optimizer/solver/solver_kp_as_input_constrained.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"

namespace PathOptimizationNS {

OsqpSolver::OsqpSolver(const ReferencePath &reference_path,
                       const VehicleState &vehicle_state,
                       const size_t &horizon) :
    horizon_(horizon),
    reference_path_(reference_path),
    vehicle_state_(vehicle_state),
    reference_interval_(0) {
    LOG(INFO) << "Optimization horizon: " << horizon;
    // Check some of the reference states to get the interval.
    const int check_num = 10;
    for (int i = 1; i < reference_path_.getSize() && i < check_num; ++i) {
        reference_interval_ = std::max(reference_interval_,
                                       reference_path_.getReferenceStates()[i].s
                                           - reference_path_.getReferenceStates()[i - 1].s);
    }
}

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