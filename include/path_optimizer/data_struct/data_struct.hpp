//
// Created by ljn on 20-1-28.
//

#ifndef PATH_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
#define PATH_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
#include <vector>
#include <memory>
#include <cfloat>

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

class Box {
 public:
    enum Dir{LEFT, RIGHT, UNKNOWN};
    Box() = delete;
    Box(double x, double y, double heading, double length, double width);
    Box(double x, double y, double heading, double length, double width, bool is_left);
    State getCenter() const { return center_; }
    double getX() const { return center_.x; }
    double getY() const { return center_.y; }
    double getHeading() const { return center_.z; }
    double getLength() const { return length_; }
    double getWidth() const { return width_; }
    double distanceTo(const State &point)  const;
    Dir getDir() const { return dir_;}
    void setDir(Box::Dir dir) { dir_ = dir;}
 private:
    State center_;
    double length_, width_;
    Dir dir_;
};

struct Circle {
    Circle() = default;
    Circle(double x, double y, double r) : x(x), y(y), r(r) {}
    double x{};
    double y{};
    double r{};
};

class BoxByCircles {
 public:
    BoxByCircles() = delete;
    BoxByCircles(const Box &box);
    const std::vector<Circle> &getCircles() const { return circles_;}
    const Box::Dir getDir() const { return dir_;}
 private:
    std::vector<Circle> circles_;
    Box::Dir dir_;
};

struct CoveringCircleBounds {
    CoveringCircleBounds() = default;
    struct SingleCircleBounds {
        SingleCircleBounds() = default;
        SingleCircleBounds &operator=(const std::vector<double> &bounds) {
            ub = bounds[0];
            lb = bounds[1];
        }
        void set(const std::vector<double> &bounds, const State &center) {
            ub = bounds[0];
            lb = bounds[1];
            x = center.x;
            y = center.y;
            heading = center.z;
        }
        double ub{}; // left
        double lb{}; // right
        double x{}, y{}, heading{};
    } c0, c1, c2, c3;
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

// Point for DP.
struct DpPoint {
    double x_, y_, heading_, s_, l_, cost_ = DBL_MAX, dir_, dis_to_obs_;
    int layer_index_, lateral_index_;
    double rough_upper_bound, rough_lower_bound;
    const DpPoint *parent_ = nullptr;
    bool is_feasible_ = true;
};

class PointComparator {
 public:
    bool operator()(APoint *a, APoint *b) {
        return a->f() > b->f();
    }
};

}
#endif //PATH_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
