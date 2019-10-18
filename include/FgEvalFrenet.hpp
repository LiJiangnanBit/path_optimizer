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
                 const hmpl::State &second_state,
                 const hmpl::State &third_state,
                 const std::vector<double> &car_geometry,
                 const std::vector<std::vector<double>> &bounds) :
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
        second_state_(second_state),
        third_state_(third_state),
        car_geometry_(car_geometry),
        bounds_(bounds) {}
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
    const hmpl::State &second_state_;
    const hmpl::State &third_state_;
    const std::vector<double> &car_geometry_;
    const std::vector<std::vector<double> > &bounds_;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        const size_t pq_range_begin = 0;
        const size_t end_heading_range_begin = pq_range_begin + N - 3;

        const size_t cons_curvature_range_begin = 1;
        const size_t cons_rear_range_begin = cons_curvature_range_begin + N - 2;
        const size_t cons_center_range_begin = cons_rear_range_begin + N - 2;
        const size_t cons_front_range_begin = cons_center_range_begin + N - 2;

        // The rest of the constraints
        AD<double> curvature_by_position_before;
        AD<double> end_heading = vars[end_heading_range_begin];
        AD<double> curvature_by_position;
        AD<double> heading;
        AD<double> heading_before;
        AD<double> ds_before;
        AD<double> ds;
        AD<double> rear_pq_before, center_pq_before, front_pq_before, psi_before, end_rear_pq, end_center_pq, end_front_pq;

        AD<double> rear_axle_to_center_circle = car_geometry_[4];
        AD<double> rear_axle_to_rear_circle = rear_axle_to_center_circle - car_geometry_[0];
        AD<double> rear_axle_to_front_circle = rear_axle_to_center_circle + car_geometry_[1];
        for (size_t i = 0; i != N - 3; ++i) {
            size_t i_for_lists = i + 3;
            AD<double> pq = vars[i];
            AD<double> pq_before_before;
            AD<double> pq_before;
            AD<double> ref_x_before_before;
            AD<double> ref_y_before_before;
            AD<double> ref_angle_before_before;
            AD<double> ref_x_before;
            AD<double> ref_y_before;
            AD<double> ref_angle_before;
            AD<double> ref_x;
            AD<double> ref_y;
            AD<double> ref_angle;

            AD<double> x_before_before;
            AD<double> y_before_before;
            AD<double> x_before;
            AD<double> y_before;
            AD<double> x;
            AD<double> y;
            if (i >= 2) {
                pq_before_before = vars[i - 2];
                pq_before = vars[i - 1];

                ref_x_before_before = seg_x_list_[i_for_lists - 2];
                ref_y_before_before = seg_y_list_[i_for_lists - 2];
                ref_angle_before_before = seg_angle_list_[i_for_lists - 2];
                ref_x_before = seg_x_list_[i_for_lists - 1];
                ref_y_before = seg_y_list_[i_for_lists - 1];
                ref_angle_before = seg_angle_list_[i_for_lists - 1];
                ref_x = seg_x_list_[i_for_lists];
                ref_y = seg_y_list_[i_for_lists];
                ref_angle = seg_angle_list_[i_for_lists];

                x_before_before = ref_x_before_before + pq_before_before * CppAD::cos(ref_angle_before_before + M_PI_2);
                y_before_before = ref_y_before_before + pq_before_before * CppAD::sin(ref_angle_before_before + M_PI_2);
                x_before = ref_x_before + pq_before * CppAD::cos(ref_angle_before + M_PI_2);
                y_before = ref_y_before + pq_before * CppAD::sin(ref_angle_before + M_PI_2);
                x = ref_x + pq * CppAD::cos(ref_angle + M_PI_2);
                y = ref_y + pq * CppAD::sin(ref_angle + M_PI_2);
            } else if (i == 0) {
                ref_x = seg_x_list_[i_for_lists];
                ref_y = seg_y_list_[i_for_lists];
                ref_angle = seg_angle_list_[i_for_lists];

                x_before_before = second_state_.x;
                y_before_before = second_state_.y;
                x_before = third_state_.x;
                y_before = third_state_.y;
                x = ref_x + pq * CppAD::cos(ref_angle + M_PI_2);
                y = ref_y + pq * CppAD::sin(ref_angle + M_PI_2);
                ds_before = sqrt(pow(x_before - x_before_before, 2) + pow(y_before - y_before_before, 2));
            } else if (i == 1) {
                pq_before = vars[i - 1];

                ref_x_before = seg_x_list_[i_for_lists - 1];
                ref_y_before = seg_y_list_[i_for_lists - 1];
                ref_angle_before = seg_angle_list_[i_for_lists - 1];
                ref_x = seg_x_list_[i_for_lists];
                ref_y = seg_y_list_[i_for_lists];
                ref_angle = seg_angle_list_[i_for_lists];

                x_before_before = third_state_.x;
                y_before_before = third_state_.y;
                x_before = ref_x_before + pq_before * CppAD::cos(ref_angle_before + M_PI_2);
                y_before = ref_y_before + pq_before * CppAD::sin(ref_angle_before + M_PI_2);
                x = ref_x + pq * CppAD::cos(ref_angle + M_PI_2);
                y = ref_y + pq * CppAD::sin(ref_angle + M_PI_2);
            }

            if (seg_x_list_[i_for_lists] - seg_x_list_[i_for_lists - 1] < 0) {
                heading = CppAD::atan2(-y + y_before, -x + x_before);
                heading_before = CppAD::atan2(-y_before + y_before_before, -x_before + x_before_before);
                ds = CppAD::fabs((x - x_before) / CppAD::cos(heading));
                curvature_by_position = (heading - heading_before) / ds_before;
                if (seg_angle_list_[i_for_lists - 1] >= 0)
                    psi_before = heading
                        - (seg_angle_list_[i_for_lists - 1] - M_PI);
                else psi_before = heading - (seg_angle_list_[i_for_lists - 1] + M_PI);
            } else {
                heading = CppAD::atan2(y - y_before, x - x_before);
                heading_before = CppAD::atan2(y_before - y_before_before, x_before - x_before_before);
                ds = CppAD::fabs((x - x_before) / CppAD::cos(heading));
                curvature_by_position = (heading - heading_before) / ds_before;
                psi_before = heading - seg_angle_list_[i_for_lists - 1];
            }
            fg[0] += cost_func_curvature_weight_ * pow(curvature_by_position, 2);
            fg[0] += cost_func_s_weight_ * pow(ds_before, 2);
            if (i != 0) {
                fg[0] +=
                    cost_func_curvature_rate_weight_ * pow(curvature_by_position - curvature_by_position_before, 2);
            }
            fg[cons_curvature_range_begin + i] = curvature_by_position;
            rear_pq_before = rear_axle_to_rear_circle * sin(psi_before) + pq_before;
            center_pq_before = rear_axle_to_center_circle * sin(psi_before) + pq_before;
            front_pq_before = rear_axle_to_front_circle * sin(psi_before) + pq_before;
            fg[cons_rear_range_begin + i] = rear_pq_before;
            fg[cons_center_range_begin + i] = center_pq_before;
            fg[cons_front_range_begin + i] = front_pq_before;
            // TODO: for skid steering vehicle, use center pq.
            fg[0] += cost_func_bound_weight_ * (1 / (pow(rear_pq_before - bounds_[i_for_lists - 1][0], 2) + 0.1)
                + 1 / (pow(rear_pq_before - bounds_[i_for_lists - 1][1], 2) + 0.1));
            curvature_by_position_before = curvature_by_position;
            ds_before = ds;
            // if this is the end state
            if (i == N - 4) {
                curvature_by_position = (end_heading - heading) / ds;
                fg[cons_curvature_range_begin + i + 1] = curvature_by_position;
                AD<double> end_psi;
                if (seg_x_list_[i_for_lists] - seg_x_list_[i_for_lists - 1] < 0) {
                    if (seg_angle_list_[i_for_lists] >= 0)
                        end_psi = end_heading
                            - (seg_angle_list_[i_for_lists] - M_PI);
                    else end_psi = end_heading - (seg_angle_list_[i_for_lists] + M_PI);
                } else {
                    end_psi = end_heading - seg_angle_list_[i_for_lists];
                }
                end_rear_pq = rear_axle_to_rear_circle * sin(end_psi) + pq;
                end_center_pq = rear_axle_to_center_circle * sin(end_psi) + pq;
                end_front_pq = rear_axle_to_front_circle * sin(end_psi) + pq;
                fg[cons_rear_range_begin + i + 1] = end_rear_pq;
                fg[cons_center_range_begin + i + 1] = end_center_pq;
                fg[cons_front_range_begin + i + 1] = end_front_pq;
            }
        }
    }
};
}

#endif //MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
