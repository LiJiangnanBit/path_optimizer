//
// Created by ljn on 19-10-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_FGEVALREFERENCESMOOTHING_HPP_
#define PATH_OPTIMIZER_INCLUDE_FGEVALREFERENCESMOOTHING_HPP_
namespace PathOptimizationNS {
using CppAD::AD;
class FgEvalReferenceSmoothing {
public:
    FgEvalReferenceSmoothing(const std::vector<double> &seg_x_list,
                             const std::vector<double> &seg_y_list,
                             const std::vector<double> &seg_s_list,
                             const int &N) :
        N(N),
        seg_s_list_(seg_s_list),
        seg_x_list_(seg_x_list),
        seg_y_list_(seg_y_list),
        curvature_weight_(10.0),
        deviation_weight_(0.001) {}
public:

    static AD<double> constraintAngle(AD<double> angle) {
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
    const double curvature_weight_, deviation_weight_;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef AD<double> ad;

    void operator()(ADvector &fg, const ADvector &vars) {
        const auto x_range_begin = 0;
        const auto y_range_begin = x_range_begin + N;
        for (size_t i = 1; i != N - 1; ++i) {
            ad last_x = vars[x_range_begin + i - 1];
            ad last_y = vars[y_range_begin + i - 1];
            ad current_x = vars[x_range_begin + i];
            ad current_y = vars[y_range_begin + i];
            ad next_x = vars[x_range_begin + i + 1];
            ad next_y = vars[y_range_begin + i + 1];
            ad ref_x = seg_x_list_[i];
            ad ref_y = seg_y_list_[i];
            // Deviation cost:
            fg[0] += deviation_weight_ * (pow(current_x - ref_x, 2) + pow(current_y - ref_y, 2));
            // Curvature cost:
            fg[0] += curvature_weight_
                * (pow(next_x + last_x - 2 * current_x, 2) + pow(next_y + last_y - 2 * current_y, 2));
            // Set constraints.
            fg[1 + i - 1] = pow(current_x - ref_x, 2) + pow(current_y - ref_y, 2);
        }
        // The last point:
        fg[0] += deviation_weight_ * (pow(vars[x_range_begin + N - 1] - seg_x_list_.back(), 2)
            + pow(vars[y_range_begin + N - 1] - seg_y_list_.back(), 2));
        fg[N - 2] = pow(vars[x_range_begin + N - 1] - seg_x_list_.back(), 2)
            + pow(vars[y_range_begin + N - 1] - seg_y_list_.back(), 2);
    }
};
}
#endif //PATH_OPTIMIZER_INCLUDE_FGEVALREFERENCESMOOTHING_HPP_
