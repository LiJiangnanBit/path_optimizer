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

struct Circle {
    Circle() = default;
    Circle(double x, double y, double r) : x(x), y(y), r(r) {}
    double x{};
    double y{};
    double r{};
};

// Point for A* search.
struct APoint {
    double x{};
    double y{};
    double s{};
    double l{};
    double g{};
    double h{};
    double dir{};
    // Layer denotes the index of the longitudinal layer that the point lies on.
    int layer{-1};
    int offset_idx{};
    double rough_upper_bound, rough_lower_bound;
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
