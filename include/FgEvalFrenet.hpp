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
    FgEvalFrenet(const tk::spline &k_s, const bool &isback,
                 const int &N, const std::vector<double> &cost_func,
                 const std::vector<double> &seg_list) :
        k_s(k_s),
        N(N),
        seg_list(seg_list),
        cost_func_cte_weight(cost_func[0]),
        cost_func_epsi_weight(cost_func[1]),
        cost_func_curvature_weight(cost_func[2]),
        cost_func_curvature_rate_weight(cost_func[3]) {}

public:
    tk::spline k_s;
    int N;
    const std::vector<double> &seg_list;
//    double ds;

    double cost_func_cte_weight;
    double cost_func_epsi_weight;
    double cost_func_curvature_weight;
    double cost_func_curvature_rate_weight;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {

        const int ps_range_begin = 0;
        const int pq_range_begin = ps_range_begin + N;
        const int psi_range_begin = pq_range_begin + N;

        const int curvature_range_begin = psi_range_begin + N;

        const double desired_q = 0.0;
        const double desired_psi = 0.0;

        fg[0] = 0.0;
        for (int t = 0; t < N; t++) {
            fg[0] += cost_func_cte_weight * pow(vars[pq_range_begin + t] - desired_q, 2);
            fg[0] += cost_func_epsi_weight * pow(vars[psi_range_begin + t] - desired_psi, 2);
        }
        //fg[0] += 100 * cost_func_cte_weight * pow(vars[pq_range_begin + N - 1] - desired_q, 2);
        // The cost function is not limited to the state, we could also include the control input! The reason we would do this is to allow us to penalize the magnitude of
        // the input as well as the change-rate. If we want to change lanes, for example, we would have a large cross-track error, but we wouldn't want to jerk the curvature
        // wheel as hard as we can. We could add the control input magnitude like this:
        for (int t = 0; t < N - 1; t++) {
            fg[0] += cost_func_curvature_weight * pow(vars[curvature_range_begin + t], 2);
        }
        // We still need to capture the change-rate of the control input to add some temporal smoothness.
        // This additional term in the cost function captures the difference between the next actuator state and the current one:

        for (int t = 0; t < N - 2; t++) {
            fg[0] += cost_func_curvature_rate_weight
                * pow(vars[curvature_range_begin + t + 1] - vars[curvature_range_begin + t], 2);
        }


        // Initial constraints
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`. This bumps up the position of all the other values.
        fg[1 + ps_range_begin] = vars[ps_range_begin];
        fg[1 + pq_range_begin] = vars[pq_range_begin];
        fg[1 + psi_range_begin] = vars[psi_range_begin];

        // The rest of the constraints
        for (int i = 0; i < N - 1; i++) {
            // Only consider the actuation at time t.
            AD<double> curvature0 = vars[curvature_range_begin + i];
            // The state at time t+1 .
            AD<double> s1 = vars[ps_range_begin + i + 1];
            AD<double> q1 = vars[pq_range_begin + i + 1];
            AD<double> psi1 = vars[psi_range_begin + i + 1];
            // The state at time t.
            AD<double> s0 = vars[ps_range_begin + i];
            AD<double> q0 = vars[pq_range_begin + i];
            AD<double> psi0 = vars[psi_range_begin + i];


//            AD<double> tmp_s = Var2Par(s0);
            AD<double> s_on_path = seg_list[i];
            AD<double> ds = seg_list[i + 1] - seg_list[i];
            AD<double> k0 = k_s(Value(s_on_path));
            AD<double> tmp_ds = ds / CppAD::cos(psi0) * (1 - q0 * k0);

            AD<double> alpha = psi0 + tmp_ds * curvature0 / 2;
            AD<double> r = 1 / curvature0;
//            AD<double> len = CppAD::sqrt(2 * pow(r, 2) * (1 - CppAD::cos(tmp_ds * curvature0)));

            AD<double> dq_ref = (1 - CppAD::cos(ds * k0)) / k0;

            fg[2 + ps_range_begin + i] = s1 - (s0 + tmp_ds);
//            fg[2 + pq_range_begin + i] = q1 - (q0 + tmp_ds * CppAD::sin(psi0));
            fg[2 + pq_range_begin + i] = q1 - (q0 + tmp_ds * CppAD::sin(alpha) - dq_ref);
            fg[2 + psi_range_begin + i] = psi1 - (psi0 + (tmp_ds * curvature0 - ds * k0));
        }
    }
};
}

#endif //MPC_PATH_OPTIMIZER__FGEVALFRENET_HPP_
