//
// Created by ljn on 20-3-24.
//

#include "path_optimizer/data_struct/vehicle_state_frenet.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"
namespace PathOptimizationNS {

VehicleState::VehicleState() :
    start_state_(new State),
    end_state_(new State),
    initial_offset_(0),
    initial_heading_error_(0) {}

VehicleState::VehicleState(const PathOptimizationNS::State &start_state,
                           const PathOptimizationNS::State &end_state,
                           double offset,
                           double heading_error) :
    start_state_(new State{start_state}),
    end_state_(new State{end_state}),
    initial_offset_(offset),
    initial_heading_error_(heading_error) {}

VehicleState::~VehicleState() {
    delete start_state_;
    delete end_state_;
}

const State& VehicleState::getStartState() const {
    return *start_state_;
}

const State& VehicleState::getEndState() const {
    return *end_state_;
}

void VehicleState::setStartState(const PathOptimizationNS::State &state) {
    *start_state_ = state;
}

void VehicleState::setEndState(const PathOptimizationNS::State &state) {
    *end_state_ = state;
}

std::vector<double> VehicleState::getInitError() const {
    return {initial_offset_, initial_heading_error_};
}

void VehicleState::setInitError(double init_offset, double init_heading_error) {
    initial_offset_ = init_offset;
    initial_heading_error_ = init_heading_error;
}

}