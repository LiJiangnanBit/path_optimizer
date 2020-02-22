//
// Created by ljn on 20-2-20.
//

#include "trajectory_optimizer/trajectory_optimizer.hpp"
#include "trajectory_optimizer/solver/solver_vc.hpp"

namespace TrajOptNS {
TrajectoryOptimizer::TrajectoryOptimizer(const std::vector<State> &points_list,
                                         const State &start_state,
                                         const State &end_state,
                                         const grid_map::GridMap &map)
    : points_list_(points_list),
      start_state_(start_state),
      end_state_(end_state),
      grid_map_(map) {}

bool TrajectoryOptimizer::solve() {
    ReferenceProvider reference_provider(points_list_, start_state_, end_state_, grid_map_.maps);
    reference_provider.getReferenceTrajectory(&reference_trajectory_);
    // Get initial input.
    SolverInput solver_input(reference_trajectory_);
    solver_input.max_time = reference_trajectory_.max_time;
    solver_input.time_interval = reference_trajectory_.time_interval;
    solver_input.horizon = reference_trajectory_.state_list.size();
    for (int i = 0; i != solver_input.horizon; ++i) {
        solver_input.reference_states.emplace_back(0,
                                                   0,
                                                   reference_trajectory_.state_list[i].s,
                                                   reference_trajectory_.state_list[i].k,
                                                   reference_trajectory_.state_list[i].v);
    }
    solver_input.updateLateralBounds(grid_map_);
    int iteration_count = 0;
    SolverVC solver_vc;
    solver_vc.init(solver_input);
//    while (iteration_count < TrajOptConfig::max_iteration_times_) {
//
//        ++iteration_count;
//    }

}

}