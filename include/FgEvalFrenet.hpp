//
// Created by ljn on 19-8-16.
//

#ifndef MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
#define MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
#include <vector>

namespace PathOptimizationNS {
using CppAD::AD;

class FgEvalFrenet {
public:
    FgEvalFrenet(const std::vector<double> &seg_x_list,
                 const std::vector<double> &seg_y_list,
                 const std::vector<double> &seg_angle_list,
                 const std::vector<double> &seg_k_list,
                 const std::vector<double> &seg_s_list,
                 const int &N,
                 const std::vector<double> &cost_func,
                 const hmpl::State &first_state,
                 const std::vector<double> &car_geometry,
                 const std::vector<std::vector<double>> &bounds,
                 const double epsi) :
        N(N),
        seg_s_list_(seg_s_list),
        seg_x_list_(seg_x_list),
        seg_y_list_(seg_y_list),
        seg_angle_list_(seg_angle_list),
        seg_k_list_(seg_k_list),
        cost_func_curvature_weight_(cost_func[0]),
        cost_func_curvature_rate_weight_(cost_func[1]),
        cost_func_bound_weight_(cost_func[2]),
        cost_func_s_weight_(cost_func[3]),
        first_state_(first_state),
        car_geometry_(car_geometry),
        bounds_(bounds),
        epsi_(epsi) {}
public:

    AD<double> constraintAngle(AD<double> angle) {
        if (angle > M_PI) {
            angle -= 2 * M_PI;
            return constraintAngle(angle);
        } else if (angle < -M_PI) {
            angle += 2 * M_PI;
            return constraintAngle(angle);
        } else {
            return angle;
        }
    }
    size_t N;
    const std::vector<double> &seg_s_list_;
    const std::vector<double> &seg_x_list_;
    const std::vector<double> &seg_y_list_;
    const std::vector<double> &seg_k_list_;
    const std::vector<double> &seg_angle_list_;

    double cost_func_curvature_weight_;
    double cost_func_curvature_rate_weight_;
    double cost_func_bound_weight_;
    double cost_func_s_weight_;

    const hmpl::State &first_state_;
    const std::vector<double> &car_geometry_;
    const std::vector<std::vector<double> > &bounds_;
    double epsi_;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        const size_t pq_range_begin = 0;
        const size_t psi_range_begin = pq_range_begin + N;
        const size_t steer_range_begin = psi_range_begin + N;
        const size_t cons_pq_range_begin = 1;
        const size_t cons_psi_range_begin = cons_pq_range_begin + N;
        const size_t cons_rear_range_begin = cons_psi_range_begin + N;
        const size_t cons_center_range_begin = cons_rear_range_begin + N;
        const size_t cons_front_range_begin = cons_center_range_begin + N;
        const size_t cons_steer_change_range_begin = cons_front_range_begin + N;
        AD<double> rear_axle_to_center_circle = car_geometry_[4];
        AD<double> rear_axle_to_rear_circle = rear_axle_to_center_circle - car_geometry_[0];
        AD<double> rear_axle_to_front_circle = rear_axle_to_center_circle + car_geometry_[1];
        AD<double> wheel_base = car_geometry_[5];
        fg[cons_pq_range_begin] = vars[pq_range_begin];
        fg[cons_psi_range_begin] = vars[psi_range_begin];
        for (size_t i = 0; i != N - 1; ++i) {
            AD<double> pq0 = vars[pq_range_begin + i];
            AD<double> pq1 = vars[pq_range_begin + i + 1];
            AD<double> psi0 = vars[psi_range_begin + i];
            AD<double> psi1 = vars[psi_range_begin + i + 1];
            AD<double> steer0 = vars[steer_range_begin + i];
            AD<double> steer1 = vars[steer_range_begin + i + 1];
            AD<double> ds = seg_s_list_[i + 1] - seg_s_list_[i];
            AD<double> next_pq = pq0 + ds * psi0;
            fg[cons_pq_range_begin + i + 1] = pq1 - next_pq;
            AD<double> steer_ref = atan(wheel_base * seg_k_list_[i]);
            AD<double> next_psi = psi0 - ds * pow(seg_k_list_[i], 2) * pq0
                + ds / wheel_base / pow(cos(steer_ref), 2) * (steer0 - steer_ref);
            fg[cons_psi_range_begin + i + 1] = psi1 - next_psi;
            AD<double> rear_pq = rear_axle_to_rear_circle * (psi0) + pq0;
            AD<double> center_pq = rear_axle_to_center_circle * (psi0) + pq0;
            AD<double> front_pq = rear_axle_to_front_circle * (psi0) + pq0;
            fg[cons_rear_range_begin + i] = rear_pq;
            fg[cons_center_range_begin + i] = center_pq;
            fg[cons_front_range_begin + i] = front_pq;
            fg[cons_steer_change_range_begin + i] = fabs(steer1 - steer0);
            if (i == N - 2) {
                rear_pq = rear_axle_to_rear_circle * (psi1) + pq1;
                center_pq = rear_axle_to_center_circle * (psi1) + pq1;
                front_pq = rear_axle_to_front_circle * (psi1) + pq1;
                fg[cons_rear_range_begin + i + 1] = rear_pq;
                fg[cons_center_range_begin + i + 1] = center_pq;
                fg[cons_front_range_begin + i + 1] = front_pq;
            }
            fg[0] += cost_func_curvature_weight_ * pow(steer0, 2);
            fg[0] += cost_func_curvature_rate_weight_ * pow(steer1 - steer0, 2);
            fg[0] += 1 * pow(pq0, 2);
        }
    }
};
}

#endif //MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
