// //
// // Created by ljn on 20-2-22.
// //

// #ifndef TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SOLVER_SOLVER_VC_HPP_
// #define TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SOLVER_SOLVER_VC_HPP_

// #include "trajectory_optimizer/solver/solver.hpp"

// namespace TrajOptNS {

// // This class implements solver using curvature and velocity as control variables.
// class SolverVC : public Solver {

// public:
//     SolverVC();
//     void init(const SolverInput &input) override ;
//     bool solve() override ;

// private:
//     void setHessianAndGradient(const SolverInput &input);
//     void setConstraintMatrix(const SolverInput &input);
//     void setBounds(Eigen::VectorXd *lower_bound, Eigen::VectorXd *upper_bound);
// };
// }
// #endif //TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_SOLVER_SOLVER_VC_HPP_
