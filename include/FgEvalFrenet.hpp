//
// Created by ljn on 19-8-16.
//

#ifndef MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
#define MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
#include <vector>

namespace MpcSmoother {
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
                 const std::vector<double> &left_bound,
                 const std::vector<double> &right_bound) :
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
        left_bound_(left_bound),
        right_bound_(right_bound) {}

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

    const hmpl::State& first_state_;
    const hmpl::State& second_state_;
    const hmpl::State& third_state_;
    const std::vector<double> &left_bound_;
    const std::vector<double> &right_bound_;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {

        const size_t pq_range_begin = 0;
        const size_t heading_range_begin = pq_range_begin + N - 3;
        const size_t ps_range_begin = heading_range_begin + 1;
        const size_t curvature_range_begin = ps_range_begin + N - 3;

        const size_t cons_heading_range_begin = 1;
        const size_t cons_curvature_range_begin = cons_heading_range_begin + 1;
        const size_t cons_ps_range_begin = cons_curvature_range_begin + N - 3;

        for (size_t t = 0; t < N - 3; t++) {
            fg[0] += cost_func_curvature_weight_ * pow(vars[curvature_range_begin + t], 2);
            fg[0] += cost_func_bound_weight_ *
                (1 / (pow((vars[pq_range_begin + t] - left_bound_[t + 3]), 2) + 0.1) +
                    1 / (pow((vars[pq_range_begin + t] - right_bound_[t + 3]), 2) + 0.1));
            fg[0] += cost_func_s_weight_ * pow(vars[ps_range_begin + t], 2);
        }
        fg[0] += cost_func_curvature_rate_weight_ * pow(vars[curvature_range_begin] - first_state_.k, 2);
        for (size_t t = 0; t < N - 4; t++) {
            fg[0] += cost_func_curvature_rate_weight_
                * pow(vars[curvature_range_begin + t + 1] - vars[curvature_range_begin + t], 2);
        }

//        AD<double> curvature_0 = vars[curvature_range_begin];
//        AD<double> curvature_1 = vars[curvature_range_begin + 1];
//        AD<double> ps_0 = vars[ps_range_begin];
//        AD<double> ps_1 = vars[ps_range_begin + 1];
//        AD<double> pq_0 = vars[pq_range_begin];
//        AD<double> pq_1 = vars[pq_range_begin + 1];
//        AD<double> ref_x_0 = seg_x_list_[2];
//        AD<double> ref_y_0 = seg_y_list_[2];
//        AD<double> ref_angle_0 = seg_angle_list_[2];
//        AD<double> ref_x_1 = seg_x_list_[3];
//        AD<double> ref_y_1 = seg_y_list_[3];
//        AD<double> ref_angle_1 = seg_angle_list_[3];


        // The rest of the constraints
        for (size_t i = 0; i != N - 3; ++i) {
            size_t i_for_lists = i + 3;
            AD<double> curvature = vars[curvature_range_begin + i];
            AD<double> ps = vars[ps_range_begin + i];
            AD<double> pq = vars[pq_range_begin + i];

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
                pq_before_before = vars[pq_range_begin + i - 2];
                pq_before = vars[pq_range_begin + i - 1];

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
            } else if (i == 1) {
                pq_before = vars[pq_range_begin + i - 1];

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

            AD<double> heading_by_position;
            if (i == N - 2) {
                heading_by_position = CppAD::atan2(y - y_before, x - x_before);
                AD<double> heading = vars[heading_range_begin];
                fg[cons_heading_range_begin] = heading - heading_by_position;
            }

            // Two methods to calculate curvature. The first method is more accurate while the second method runs faster.
            if (seg_x_list_[i_for_lists] - seg_x_list_[i_for_lists - 1] < 0) {
                AD<double> curvature_by_position;
                AD<double> heading = CppAD::atan2(-y + y_before, -x + x_before);
                AD<double> heading_before = CppAD::atan2(-y_before + y_before_before, -x_before + x_before_before);
                AD<double> ds_before = CppAD::fabs((x_before - x_before_before) / CppAD::cos(heading_before));
                curvature_by_position = (heading - heading_before) / ds_before;
                fg[cons_curvature_range_begin + i] = curvature - curvature_by_position;
                fg[cons_ps_range_begin + i] = ps - ds_before;
            } else {
                AD<double> curvature_by_position;
                AD<double> heading = CppAD::atan2(y - y_before, x - x_before);
                AD<double> heading_before = CppAD::atan2(y_before - y_before_before, x_before - x_before_before);
                AD<double> ds_before = CppAD::fabs((x_before - x_before_before) / CppAD::cos(heading_before));
                curvature_by_position = (heading - heading_before) / ds_before;
                fg[cons_curvature_range_begin + i] = curvature - curvature_by_position;
                fg[cons_ps_range_begin + i] = ps - ds_before;
            }

//            AD<double> ref_ds_before = seg_s_list_[i] - seg_s_list_[i_before_before];
//            AD<double> psi_before = (pq_before - pq_before_before) / ref_ds_before;
//            AD<double> ps_before = ref_ds_before / CppAD::cos(psi_before) * (1 - pq_before_before * seg_k_list_[i_before_before]);
//            AD<double> ref_ds_after = seg_s_list_[i_before] - seg_s_list_[i];
//            AD<double> psi_after = (pq_after - pq_before) / ref_ds_after;
//            AD<double> curvature_by_position = (psi_after - psi_before + constraintAngle(ref_angle_before - ref_angle_before_before)) / ps_before;

        }
    }
};
}

#endif //MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
