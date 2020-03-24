//
// Created by ljn on 20-3-24.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_VEHICLE_STATE_FRENET_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_VEHICLE_STATE_FRENET_HPP_
#include <vector>

namespace PathOptimizationNS {

class State;

class VehicleState {
 public:
    VehicleState();
    VehicleState(const State &start_state,
                 const State &end_state,
                 double offset = 0,
                 double heading_error = 0);
    ~VehicleState();
    const State &getStartState() const;
    const State &getEndState() const;
    void setStartState(const State &state);
    void setEndState(const State &state);
    std::vector<double> getInitError() const;
    void setInitError(double init_offset, double init_heading_error);

 private:
    // Initial state.
    State *start_state_;
    // Target state.
    State *end_state_;
    // Initial error with reference line.
    double initial_offset_{};
    double initial_heading_error_{};
};

}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_VEHICLE_STATE_FRENET_HPP_
