//
// Created by ljn on 20-3-23.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_HPP_
#include <memory>
#include <vector>

namespace PathOptimizationNS {
class Map;
class Config;
class State;
namespace tk {
class spline;
}
class ReferencePathImpl;

class ReferencePath {
 public:
    ReferencePath();
    const tk::spline &getXS() const;
    const tk::spline &getYS() const;
    double getXS(double s) const;
    double getYS(double s) const;
    void setSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    void clear();
    std::size_t getSize() const;
    bool trimStates();
    double getLength() const;
    void setLength(double s);
    const std::vector<State> &getReferenceStates() const;
    const std::vector<CoveringCircleBounds> &getBounds() const;
    const std::vector<double> &getMaxKList() const;
    const std::vector<double> &getMaxKpList() const;
    // Set reference_states_ directly, only used in solveWithoutSmoothing.
    void setReference(const std::vector<State> &reference);
    void setReference(const std::vector<State> &&reference);
    // Calculate upper and lower bounds for each covering circle.
    void updateBounds(const Map &map, const Config &config);
    // If the reference_states_ have speed and acceleration information, call this func to calculate
    // curvature and curvature rate bounds.
    void updateLimits(const Config &config);
    // Calculate reference_states_ from x_s_ and y_s_, given delta s.
    bool buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger);
 private:
    std::shared_ptr<ReferencePathImpl> reference_path_impl_;
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_HPP_
