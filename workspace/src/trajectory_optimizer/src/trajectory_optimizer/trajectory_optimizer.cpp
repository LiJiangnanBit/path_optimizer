//
// Created by ljn on 20-2-20.
//

#include <fstream>
#include "trajectory_optimizer/trajectory_optimizer.hpp"
#include "trajectory_optimizer/solver/solver_ipopt_kpvp.hpp"

namespace TrajOptNS {

TrajectoryOptimizer::TrajectoryOptimizer(const std::vector<State> &points_list,
                                         const State &start_state,
                                         const State &end_state,
                                         const grid_map::GridMap &map)
    : points_list_(points_list),
      start_state_(start_state),
      end_state_(end_state),
      grid_map_(map),
      reference_trajectory_(new FrenetTrajectory())
      {
      }

bool TrajectoryOptimizer::solve(std::vector<State> *result_trajectory) {
    ReferenceProvider reference_provider(points_list_, start_state_, end_state_, grid_map_.maps);
    reference_provider.getReferenceTrajectory(reference_trajectory_);
    std::shared_ptr<SolverInput> solver_input_ptr{new SolverInput};
    solver_input_ptr->init(reference_trajectory_, start_state_, end_state_);
    solver_input_ptr->updateLateralBounds(grid_map_);
    std::shared_ptr<Solver> solver_ptr{new SolverIpoptKpvp};
    solver_ptr->init(solver_input_ptr);
    solver_ptr->solve(result_trajectory);
    std::ofstream ref_v_out, v_out;
    ref_v_out.open("/home/ljn/path-optimizer/workspace/src/trajectory_optimizer/ref_v.csv");
    v_out.open("/home/ljn/path-optimizer/workspace/src/trajectory_optimizer/v.csv");
    ref_v_out << "n,v" << std::endl;
    v_out << "n,v" << std::endl;
    for (size_t i = 0; i != result_trajectory->size(); ++i) {
        ref_v_out << i << "," << reference_trajectory_->state_list[i].v << std::endl;
        v_out << i << "," << result_trajectory->at(i).v << std::endl;
    }

}

}