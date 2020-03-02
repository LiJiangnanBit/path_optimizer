 //
 // Created by ljn on 20-2-22.
 //

 #include <fstream>
 #include<iomanip>
 #include <path_optimizer/tools/tools.hpp>
 #include "trajectory_optimizer/solver/solver_vc.hpp"

 using PathOptimizationNS::getHeading;
 namespace TrajOptNS {

 SolverVC::SolverVC() :
     Solver() {}

 void SolverVC::init(const TrajOptNS::SolverInput &input) {
 //    horizon_ = input.horizon;
     horizon_ = 4;
     setHessianAndGradient(input);
     setConstraintMatrix(input);
 }

 bool SolverVC::solve() {
     auto t1 = std::clock();
     solver_.settings()->setVerbosity(false);
     solver_.settings()->setWarmStart(true);
     solver_.data()->setNumberOfVariables(6 * horizon_ - 2);
     solver_.data()->setNumberOfConstraints(15 * horizon_ - 4);
     // Input to solver.
     if (!solver_.data()->setHessianMatrix(hessian_)) return false;
     if (!solver_.data()->setGradient(gradient_)) return false;
     if (!solver_.data()->setLinearConstraintsMatrix(linear_matrix_)) return false;
     if (!solver_.data()->setLowerBound(lower_bound_)) return false;
     if (!solver_.data()->setUpperBound(upper_bound_)) return false;
     // Solve.
     if (!solver_.initSolver()) return false;
 //    if (!solver_.solve()) return false;
     bool ok = solver_.solve();
     auto t2 = std::clock();
     std::cout << "ok? " << ok << std::endl;
     if (ok) {
         std::cout << double(t2 - t1) / CLOCKS_PER_SEC << std::endl;
         Eigen::VectorXd solution = solver_.getSolution();
 //        for (int i = 0; i != horizon_ - 1; ++i) {
 //            std::cout << solution(3 * horizon_ + 2 * i + 1) << std::endl;
 //        }
 //        std::cout << "ddddddddddddd" << std::endl;
 //        for (int i = 0; i != horizon_ - 1; ++i) {
 //            std::cout << solution(3 * horizon_ + 2 * i) << std::endl;
 //        }
         std::cout << "solution :" << std::endl;
         std::cout << solution << std::endl;
     }
 }

 void SolverVC::setHessianAndGradient(const SolverInput &input) {
     double w_c{2}; // TODO: use config.
     double w_cr{10};
     double w_a(500);
     double w_j{200};
     double w_e{10};
     double w_s{5};
     double w_f(1);

     // Calculate hessian matrix.
     const int state_size{3 * horizon_};
     const int control_size{2 * (horizon_ - 1)};
     const int slack_size{horizon_};
     const auto hessian_size = state_size + control_size + slack_size;
     Eigen::MatrixXd hessian{Eigen::MatrixXd::Constant(hessian_size, hessian_size, 0)};
     // Curvature part.
     for (size_t i = state_size; i != state_size + control_size; i += 2) {
         hessian(i, i) += w_c;
     }
     // Curvature change part.
     Eigen::Matrix4d curvature_change_part;
     curvature_change_part <<
                           1, 0, -1, 0,
         0, 0, 0, 0,
         -1, 0, 1, 0,
         0, 0, 0, 0;
     for (size_t i = state_size; i != state_size + control_size - 2; i += 2) {
         hessian.block(i, i, 4, 4) += curvature_change_part * w_cr;
     }
     // acc part.
     Eigen::Matrix<double, 4, 1> a1;
     a1 << 0, -1, 0, 1;
     Eigen::Matrix<double, 4, 4> acc_part{a1 * a1.transpose()};
     for (size_t i = state_size; i != state_size + control_size - 2; i += 2) {
         hessian.block(i, i, 4, 4) += acc_part * w_a;
     }
     // Jerk part.
     Eigen::Matrix<double, 6, 1> j1;
     j1 << 0, 1, 0, -2, 0, 1;
     Eigen::Matrix<double, 6, 6> jerk_part{j1 * j1.transpose()};
     for (size_t i = state_size; i != state_size + control_size - 4; i += 2) {
         hessian.block(i, i, 6, 6) += jerk_part * w_j;
     }
 //    // Error part.
 //    for (size_t i = 0; i != state_size + control_size; ++i) {
 //        hessian(i, i) += w_e;
 //    }
     // Slack part.
     for (size_t i = state_size + control_size; i != hessian_size; ++i) {
         hessian(i, i) += w_s;
     }
     hessian_ = 2 * hessian.sparseView();
 //std::cout << hessian << std::endl;
     // Calculate gradient.
 //    Eigen::VectorXd ref_state;
 //    ref_state.resize(hessian_size);
 //    for (int i = 0; i != horizon_; ++i) {
 //
 //        ref_state(3 * i) = input.reference_states[i].ey;
 //        ref_state(3 * i + 1) = input.reference_states[i].ephi;
 //        ref_state(3 * i + 2) = input.reference_states[i].s;
 //        if (i != horizon_ - 1) {
 //            ref_state(2 * i + state_size) = input.reference_states[i].k;
 //            ref_state(2 * i + 1 + state_size) = input.reference_states[i].v;
 //        }
 //    }
 //    gradient_ = -2 * w_e * ref_state;
     gradient_ = Eigen::VectorXd::Constant(hessian_size, 0);
 }

 void SolverVC::setConstraintMatrix(const SolverInput &input) {
     const size_t cons_size = 15 * horizon_ - 4;
     const size_t variable_size = 6 * horizon_ - 2;
     Eigen::MatrixXd cons_matrix{Eigen::MatrixXd::Constant(cons_size, variable_size, 0)};
     const auto &ref = input.reference_states;
     // Transition part.
     std::vector<Eigen::Vector3d> c_list; // Needed in bounds.
     auto iter = input.reference_trajectory.state_list.begin() + 1;
     for (size_t i = 0; i != horizon_ - 1; ++i) {
         const double ref_traj_k{getHeading(input.reference_trajectory.x_s, input.reference_trajectory.y_s, ref[i].s)};
         cons_matrix.block(3 * i, 3 * i, 3, 3) = -Eigen::Matrix3d::Identity();
         // Get transition matrix.
         Eigen::Matrix3d a{Eigen::Matrix3d::Constant(0)};
         a(0, 1) = ref[i].v * cos(ref[i].ephi);
         a(2, 0) = ref[i].v * ref_traj_k * cos(ref[i].ephi) / pow(1 - ref[i].ey * ref_traj_k, 2);
         a(2, 1) = ref[i].v * sin(ref[i].ephi) / (1 - ref[i].ey * ref_traj_k);
         Eigen::MatrixXd b{Eigen::MatrixXd::Constant(3, 2, 0)};
         b(0, 1) = sin(ref[i].ephi);
         b(1, 0) = ref[i].v;
         b(1, 1) = ref[i].k;
         b(2, 1) = cos(ref[i].ephi) / (1 - ref[i].ey * ref_traj_k);
         // c is needed later in bounds.
         const double &tmp_s{ref[i].s};
         while (iter->s <= tmp_s && iter != input.reference_trajectory.state_list.end() - 1) {
             ++iter;
         }
         double v_small{(iter - 1)->v}, v_large{iter->v};
         double weight_large{(iter->s - tmp_s) / (iter->s - (iter - 1)->s)};
         const double ref_traj_v{v_large * weight_large + v_small * (1 - weight_large)};
         Eigen::Vector3d c;
         c <<
           ref[i].v * sin(ref[i].ephi),
             ref[i].v * ref[i].k - ref[i].v - ref_traj_k * ref_traj_v,
             ref[i].v * cos(ref[i].ephi) / (1 - ref[i].ey * ref_traj_k);
         Eigen::Matrix<double, 3, 1> ref_state;
         ref_state << ref[i].ey, ref[i].ephi, ref[i].s;
         Eigen::Matrix<double, 2, 1> ref_control;
         ref_control << ref[i].k, ref[i].v;
         c -= a * ref_state + b * ref_control;
         c_list.emplace_back(c);
         // Fill in.
         cons_matrix.block(3 * (i + 1), 3 * i, 3, 3) = a * input.time_interval + Eigen::Matrix3d::Identity();
         cons_matrix.block(3 * (i + 1), 3 * horizon_ + 2 * i, 3, 2) = b * input.time_interval;
     }
     cons_matrix.block(3 * (horizon_ - 1), 3 * (horizon_ - 1), 3, 3) = -Eigen::Matrix3d::Identity();
     // Control variables part.
     cons_matrix.block(3 * horizon_, 3 * horizon_, 2 * (horizon_ - 1), 2 * (horizon_ - 1)) =
         Eigen::MatrixXd::Identity(2 * (horizon_ - 1), 2 * (horizon_ - 1));
     cons_matrix.block(5 * horizon_ - 2, 5 * horizon_ - 2, horizon_, horizon_) =
         Eigen::MatrixXd::Identity(horizon_, horizon_);
     // Collision part.
     Eigen::Matrix<double, 4, 3> collision;
     collision <<
               1.0, TrajOptConfig::d1_ + TrajOptConfig::rear_axle_to_center_distance_, 0,
         1.0, TrajOptConfig::d2_ + TrajOptConfig::rear_axle_to_center_distance_, 0,
         1.0, TrajOptConfig::d3_ + TrajOptConfig::rear_axle_to_center_distance_, 0,
         1.0, TrajOptConfig::d4_ + TrajOptConfig::rear_axle_to_center_distance_, 0;
     for (size_t i = 0; i != horizon_; ++i) {
         cons_matrix.block(6 * horizon_ - 2 + 4 * i, 3 * i, 4, 3) = collision;
         cons_matrix.block(6 * horizon_ - 2 + 4 * i, 5 * horizon_ - 2 + i, 4, 1) =
             Eigen::Matrix<double, 4, 1>::Constant(-1);
         cons_matrix.block(10 * horizon_ - 2 + 4 * i, 3 * i, 4, 3) = collision;
         cons_matrix.block(10 * horizon_ - 2 + 4 * i, 5 * horizon_ - 2 + i, 4, 1) =
             Eigen::Matrix<double, 4, 1>::Constant(1);
     }
     // Acc part.
     for (size_t i = 0; i != horizon_ - 2; ++i) {
         cons_matrix(14 * horizon_ - 2 + i, 3 * horizon_ + 1 + 2 * i) = -1.0 / input.time_interval;
         cons_matrix(14 * horizon_ - 2 + i, 3 * horizon_ + 1 + 2 * (i + 1)) = 1.0 / input.time_interval;
     }
     linear_matrix_ = cons_matrix.sparseView();
 //    std::ofstream out;
 //    out.open("/home/ljn/桌面/constraint.txt");
 //    Eigen::IOFormat OctaveFmt(3, 0,", ", ";\n","", "", "", "");
 //    out << std::setw(3) << cons_matrix.format(OctaveFmt) << std::endl;

     // Set bounds.
     Eigen::VectorXd lower_bound{Eigen::VectorXd::Constant(cons_size, 0)};
     Eigen::VectorXd upper_bound{Eigen::VectorXd::Constant(cons_size, 0)};
     // Transition part.
 //    auto iter = input.reference_trajectory.state_list.begin() + 1;
     for (size_t i = 0; i != horizon_ - 1; ++i) {
 //        double tmp_s = ref[i].s;
 //        while (iter->s <= tmp_s && iter != input.reference_trajectory.state_list.end() - 1) {
 //            ++iter;
 //        }
 //        double v_small{(iter - 1)->v}, v_large{iter->v};
 //        double weight_large{(iter->s - tmp_s) / (iter->s - (iter - 1)->s)};
 //        const double ref_traj_v{v_large * weight_large + v_small * (1 - weight_large)};
 //        const double ref_traj_k{getHeading(input.reference_trajectory.x_s, input.reference_trajectory.y_s, tmp_s)};
 //        Eigen::Vector3d c;
 //        c <<
 //          ref[i].v * sin(ref[i].ephi),
 //            ref[i].v * ref[i].k - ref[i].v - ref_traj_k * ref_traj_v,
 //            ref[i].v * cos(ref[i].ephi) / (1 - ref[i].ey * ref_traj_k);
         lower_bound.block(3 * i + 3, 0, 3, 1) = -input.time_interval * c_list[i];
         upper_bound.block(3 * i + 3, 0, 3, 1) = -input.time_interval * c_list[i];
     }
     // Control part.
     for (size_t i = 0; i != horizon_ - 1; ++i) {
         lower_bound(3 * horizon_ + 2 * i) = -0.2;//-tan(TrajOptConfig::max_steer_angle_) / TrajOptConfig::wheel_base_;
         upper_bound(3 * horizon_ + 2 * i) = 0.2;//tan(TrajOptConfig::max_steer_angle_) / TrajOptConfig::wheel_base_;
         lower_bound(3 * horizon_ + 2 * i + 1) = 0;
         upper_bound(3 * horizon_ + 2 * i + 1) = 15;
     }
     for (size_t i = 0; i != horizon_; ++i) {
         lower_bound(5 * horizon_ - 2 + i) = 0;
         upper_bound(5 * horizon_ - 2 + i) = OsqpEigen::INFTY;
     }
     lower_bound(3 * horizon_) = input.reference_states[0].k;
     upper_bound(3 * horizon_) = input.reference_states[0].k;
     lower_bound(3 * horizon_ + 1) = input.reference_states[0].v;
     upper_bound(3 * horizon_ + 1) = input.reference_states[0].v;
     // Collision part.
     lower_bound.block(6 * horizon_ - 2, 0, 4 * horizon_, 1) =
         Eigen::VectorXd::Constant(4 * horizon_, -OsqpEigen::INFTY);
     upper_bound.block(10 * horizon_ - 2, 0, 4 * horizon_, 1) =
         Eigen::VectorXd::Constant(4 * horizon_, OsqpEigen::INFTY);
     for (size_t i = 0; i != horizon_; ++i) {
         Eigen::Vector4d collision_ub, collision_lb;
         collision_ub <<
                      input.bounds[i].c0.ub,
             input.bounds[i].c1.ub,
             input.bounds[i].c2.ub,
             input.bounds[i].c3.ub;
         collision_lb <<
                      input.bounds[i].c0.lb,
             input.bounds[i].c1.lb,
             input.bounds[i].c2.lb,
             input.bounds[i].c3.lb;
         upper_bound.block(6 * horizon_ - 2 + 4 * i, 0, 4, 1) = collision_ub;
         lower_bound.block(10 * horizon_ - 2 + 4 * i, 0, 4, 1) = collision_lb;
     }
     // Acc part.
     lower_bound.block(14 * horizon_ - 2, 0, horizon_ - 2, 1) =
         Eigen::VectorXd::Constant(horizon_ - 2, TrajOptConfig::max_lon_dacc_);
     upper_bound.block(14 * horizon_ - 2, 0, horizon_ - 2, 1) =
         Eigen::VectorXd::Constant(horizon_ - 2, TrajOptConfig::max_lon_acc_);
     lower_bound_ = lower_bound;
     upper_bound_ = upper_bound;



 //    std::cout << "cons: " << std::endl;
 //    std::cout << cons_matrix << std::endl << std::endl;
     std::cout << "u and l bound" << std::endl;
     std::cout << upper_bound << std::endl << std::endl;
     std::cout << lower_bound << std::endl;
 }

 }