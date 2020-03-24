//
// Created by ljn on 20-1-28.
//

#ifndef PATH_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
#define PATH_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
#include <vector>
#include <memory>

namespace PathOptimizationNS {
// Standard point struct.
struct State {
  State() = default;
  State(double x, double y, double z = 0, double k = 0, double s = 0, double v = 0, double a = 0) :
      x(x),
      y(y),
      z(z),
      k(k),
      s(s),
      v(v),
      a(a) {}
  double x{};
  double y{};
  double z{}; // Heading.
  double k{}; // Curvature.
  double s{};
  double v{};
  double a{};
};

struct CoveringCircleBounds {
  struct SingleCircleBounds {
    SingleCircleBounds &operator=(std::vector<double> &bounds) {
        ub = bounds[0];
        lb = bounds[1];
    }
    double ub{}; // left
    double lb{}; // right
  } c0, c1, c2, c3;
};

//class ReferencePath {
// public:
//    const tk::spline &getXS() const {
//        return x_s_;
//    }
//    const tk::spline &getYS() const {
//        return y_s_;
//    }
//    void setSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s) {
//        x_s_ = x_s;
//        y_s_ = y_s;
//        max_s_ = max_s;
//    }
//    void clear() {
//        max_s_ = 0;
//        if (reference_states_) reference_states_->clear();
//        bounds_.clear();
//        max_k_list_.clear();
//        max_kp_list_.clear();
//    }
//    // Set reference_states_ directly, only used in solveWithoutSmoothing.
//    void setReference(const std::shared_ptr<std::vector<State>> &reference) {
//        reference_states_ = reference;
//        use_spline_ = false;
//    }
//    // Calculate upper and lower bounds for each covering circle.
//    void updateBounds(const Map &map, const Config &config);
//    // If the reference_states_ have speed and acceleration information, call this func to calculate
//    // curvature and curvature rate bounds.
//    void updateLimits(const Config &config);
//    // Calculate reference_states_ from x_s_ and y_s_, given delta s.
//    bool buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger);
//
// private:
//    std::vector<double> getClearanceWithDirectionStrict(const PathOptimizationNS::State &state,
//                                                        const PathOptimizationNS::Map &map,
//                                                        double radius) const;
//    bool use_spline_{true};
//    // Reference path spline representation.
//    tk::spline x_s_;
//    tk::spline y_s_;
//    double max_s_{};
//    // Divided smoothed path info.
//    std::shared_ptr<std::vector<State>> reference_states_;
//    std::vector<CoveringCircleBounds> bounds_;
//    std::vector<double> max_k_list_;
//    std::vector<double> max_kp_list_;
//};

struct Circle {
    Circle() = default;
    Circle(double x, double y, double r) : x(x), y(y), r(r) {}
    double x{};
    double y{};
    double r{};
};

struct VehicleState {
    VehicleState() = default;
    VehicleState(const State &start_state,
                 const State &end_state,
                 double offset = 0,
                 double heading_error = 0) :
        start_state_(start_state),
        end_state_(end_state),
        initial_offset_(offset),
        initial_heading_error_(heading_error) {}
    // Initial state.
    State start_state_;
    // Target state.
    State end_state_;
    // Initial error with reference line.
    double initial_offset_{};
    double initial_heading_error_{};
};

// Point for A* search.
struct APoint {
    double x{};
    double y{};
    double s{};
    double l{};
    double g{};
    double h{};
    // Layer denotes the index of the longitudinal layer that the point lies on.
    int layer{-1};
    double offset{};
    bool is_in_open_set{false};
    APoint *parent{nullptr};
    inline double f() {
        return g + h;
    }
};

class PointComparator {
public:
    bool operator()(APoint *a, APoint *b) {
        return a->f() > b->f();
    }
};

}
#endif //PATH_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
