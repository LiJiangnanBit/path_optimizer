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
                 const std::vector<double> &seg_s_list,
                 const int &N,
                 const std::vector<double> &cost_func) :
        N(N),
        seg_s_list_(seg_s_list),
        seg_x_list_(seg_x_list),
        seg_y_list_(seg_y_list),
        seg_angle_list_(seg_angle_list),
        cost_func_cte_weight_(cost_func[0]),
        cost_func_epsi_weight_(cost_func[1]),
        cost_func_curvature_weight_(cost_func[2]),
        cost_func_curvature_rate_weight_(cost_func[3]) {}

public:
    size_t N;
    const std::vector<double> &seg_s_list_;
    const std::vector<double> &seg_x_list_;
    const std::vector<double> &seg_y_list_;
    const std::vector<double> &seg_angle_list_;

    double cost_func_cte_weight_;
    double cost_func_epsi_weight_;
    double cost_func_curvature_weight_;
    double cost_func_curvature_rate_weight_;

    // Set angle range to -pi ~ pi.
    template<typename DOUBLE_TYPE>
    inline DOUBLE_TYPE constraintAngle(DOUBLE_TYPE angle) {
        if (angle > M_PI) {
            angle -= 2 * M_PI;
            return constraintAngle(angle);
        } else if (angle < -M_PI) {
            angle += 2 * M_PI;
            return constraintAngle(angle);
        } else{
            return angle;
        }
    }

    AD<double> getPointCurvature(const AD<double> &x1,
                                 const AD<double> &y1,
                                 const AD<double> &x2,
                                 const AD<double> &y2,
                                 const AD<double> &x3,
                                 const AD<double> &y3) {
        AD<double> a, b, c;
        AD<double> delta_x, delta_y;
        AD<double> s;
        AD<double> A;
        AD<double> curv;
        AD<double> rotate_direction;

        delta_x = x2 - x1;
        delta_y = y2 - y1;
        a = CppAD::sqrt(CppAD::pow(delta_x, 2.0) + CppAD::pow(delta_y, 2.0));

        delta_x = x3 - x2;
        delta_y = y3 - y2;
        b = CppAD::sqrt(CppAD::pow(delta_x, 2.0) + CppAD::pow(delta_y, 2.0));

        delta_x = x1 - x3;
        delta_y = y1 - y3;
        c = CppAD::sqrt(CppAD::pow(delta_x, 2.0) + CppAD::pow(delta_y, 2.0));

        s = (a + b + c) / 2.0;
        A = CppAD::sqrt(CppAD::fabs(s * (s - a) * (s - b) * (s - c)));
        curv = 4 * A / (a * b * c);

        rotate_direction = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
        if (rotate_direction < 0) {
            curv = -curv;
        }
        return curv;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {

        const size_t pq_range_begin = 0;
        const size_t heading_range_begin = pq_range_begin + N;
        const size_t curvature_range_begin = heading_range_begin + N - 1;

        const size_t cons_pq_range_begin = 1;
        const size_t cons_heading_range_begin = cons_pq_range_begin + 2;
        const size_t cons_curvature_range_begin = cons_heading_range_begin + N - 1;

        // The cost function is not limited to the state, we could also include the control input! The reason we would do this is to allow us to penalize the magnitude of
        // the input as well as the change-rate. If we want to change lanes, for example, we would have a large cross-track error, but we wouldn't want to jerk the curvature
        // wheel as hard as we can. We could add the control input magnitude like this:
        for (int t = 0; t < N - 2; t++) {
            fg[0] += cost_func_curvature_weight_ * pow(vars[curvature_range_begin + t], 2);
        }
        // We still need to capture the change-rate of the control input to add some temporal smoothness.
        // This additional term in the cost function captures the difference between the next actuator state and the current one:

        for (int t = 0; t < N - 3; t++) {
            fg[0] += cost_func_curvature_rate_weight_
                * pow(vars[curvature_range_begin + t + 1] - vars[curvature_range_begin + t], 2);
        }

        fg[cons_pq_range_begin] = vars[pq_range_begin];
        fg[cons_pq_range_begin + 1] = vars[pq_range_begin + 1];
        fg[cons_heading_range_begin] = vars[heading_range_begin];

        // The rest of the constraints
        for (size_t i = 1; i <= N - 2; i++) {
            size_t i_before = i - 1;
            size_t i_after = i + 1;
            AD<double> heading = vars[heading_range_begin + i];
            AD<double> heading_before = vars[heading_range_begin + i_before];

            AD<double> curvature = vars[curvature_range_begin + i];

            AD<double> pq_before = vars[pq_range_begin + i_before];
            AD<double> pq = vars[pq_range_begin + i];
            AD<double> pq_after = vars[pq_range_begin + i_after];

            AD<double> ref_x_before = seg_x_list_[i_before];
            AD<double> ref_y_before = seg_y_list_[i_before];
            AD<double> ref_angle_before = seg_angle_list_[i_before];
            AD<double> ref_x = seg_x_list_[i];
            AD<double> ref_y = seg_y_list_[i];
            AD<double> ref_angle = seg_angle_list_[i];
            AD<double> ref_x_after = seg_x_list_[i_after];
            AD<double> ref_y_after = seg_y_list_[i_after];
            AD<double> ref_angle_after = seg_angle_list_[i_after];

            AD<double> x_before = ref_x_before + pq_before * CppAD::cos(constraintAngle(ref_angle_before + M_PI_2));
            AD<double> y_before = ref_y_before + pq_before * CppAD::sin(constraintAngle(ref_angle_before + M_PI_2));
            AD<double> x = ref_x + pq * CppAD::cos(constraintAngle(ref_angle + M_PI_2));
            AD<double> y = ref_y + pq * CppAD::sin(constraintAngle(ref_angle + M_PI_2));
            AD<double> ds = CppAD::sqrt(CppAD::pow(x - x_before, 2) + CppAD::pow(y - y_before, 2));
//
//            AD<double> ds = CppAD::fabs((x - x_before) / CppAD::cos(heading_before));

            AD<double> x_after = ref_x_after + pq_after * CppAD::cos(constraintAngle(ref_angle_after + M_PI_2));
            AD<double> y_after = ref_y_after + pq_after * CppAD::sin(constraintAngle(ref_angle_after + M_PI_2));
//            AD<double> ds_after = CppAD::sqrt(CppAD::pow(x - x_after, 2) + CppAD::pow(y - y_after, 2));

            AD<double> heading_by_position = CppAD::atan2(y_after - y, x_after - x);
//            AD<double> psi_before = CppAD::atan((pq - pq_before) / (seg_s_list_[i] - seg_s_list_[i_before]));
//            AD<double> psi = CppAD::atan((pq_after - pq) / (seg_s_list_[i_after] - seg_s_list_[i]));
//            AD<double> curvature_by_position = (psi - psi_before) / ds;
//            AD<double> curvature_by_position = fabs(heading - heading_before) < M_PI ? (heading - heading_before) / ds :
//                                               (heading - heading_before > M_PI ? (heading - heading_before - 2 * M_PI) :
//                                                (heading - heading_before + 2 * M_PI));
//            AD<double> angle_diff_before = CppAD::atan((y - y_before) / (x - x_before));
//            AD<double> angle_diff = CppAD::atan((y_after - y) / (x_after - x));
//            AD<double> curvature_by_position = (angle_diff - angle_diff_before) / ds;
//            AD<double> curvature_by_position = CppAD::acos(CppAD::cos(heading - heading_before)) / ds;
//            AD<double> curvature_by_position = CppAD::acos(((x - x_before) * (x_after - x) + (y - y_before) * (y_after - y)) / ds / ds_after) / ds;
//            AD<double> sign = (x - x_before) / CppAD::fabs(x - x_before);
//            AD<double>
            AD<double> curvature_by_position;
            if (seg_x_list_[i] - seg_x_list_[i_before] < 0) {
                AD<double> new_heading = CppAD::atan2(-y_after + y, -x_after + x);
                AD<double> new_heading_before = CppAD::atan2(-y + y_before, -x + x_before);
                curvature_by_position = (new_heading - new_heading_before) / ds;
            } else {
                curvature_by_position = (heading - heading_before) / ds;
            }

            fg[cons_heading_range_begin + i] = heading - heading_by_position;
            fg[cons_curvature_range_begin + i - 1] = curvature - curvature_by_position;
        }
    }
};
}

#endif //MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
