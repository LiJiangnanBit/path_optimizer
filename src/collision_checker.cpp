//
// Created by yangt on 19-5-8.
//
#include "../include/collosion_checker.hpp"

namespace PathOptimizationNS {

CollisionChecker::CollisionChecker(const hmpl::InternalGridMap &in_gm)
    : in_gm_(in_gm),
      car_() {
}

bool CollisionChecker::isSinglePathCollisionFree(std::vector<hmpl::State> *curve) {
    for (auto &state_itr : *curve) {
        if (!this->isSingleStateCollisionFree(state_itr)) {
            state_itr.v = 0;  // collision
            // if we return here, state.v will have 0 initial value, we don't
            // need to update them manually
            return false;
        } else {
            // we checked, it's collision-free for sure.
            // create a feild for collison in State struct
            state_itr.v = 1;  // collision-free
        }
    }
    return true;
}

bool CollisionChecker::isSingleStateCollisionFree(const hmpl::State &current) {
    // get the footprint circles based on current vehicle state in global frame
    std::vector<hmpl::Circle> footprint =
        this->car_.getCurrentCenters(current);
    // footprint checking
    for (auto &circle_itr : footprint) {
        grid_map::Position pos(circle_itr.position.x,
                               circle_itr.position.y);
        // complete collision checking
        if (this->in_gm_.maps.isInside(pos)) {
            double clearance = this->in_gm_.getObstacleDistance(pos);
            if (clearance < circle_itr.r) {  // collision
                // less than circle radius, collision
                return false;
            }
        } else {
            // beyond boundaries , collision
            return false;
        }
    }
    // all checked, current state is collision-free
    return true;
}

bool CollisionChecker::isSinglePathCollisionFreeImproved(std::vector<hmpl::State> *curve) {
    for (auto &state_itr : *curve) {
        if (!this->isSingleStateCollisionFreeImproved(state_itr)) {
            // collision
            state_itr.v = 0;
            return false;
        } else {
            // collision-free
            state_itr.v = 1.0;
        }
    }
    return true;
}

bool CollisionChecker::isSingleStateCollisionFreeImproved(const hmpl::State &current) {
    // get the bounding circle position in global frame
    hmpl::Circle bounding_circle = this->car_.getBoundingCircle(current);

    grid_map::Position pos(bounding_circle.position.x,
                           bounding_circle.position.y);
    if (this->in_gm_.maps.isInside(pos)) {
        double clearance = this->in_gm_.getObstacleDistance(pos);
        if (clearance < bounding_circle.r) {
            // the big circle is not collision-free, then do an exact
            // collision checking
            return (this->isSingleStateCollisionFree(current));
        } else { // collision-free
            return true;
        }
    } else {  // beyond the map boundary
        return false;
    }
}

void CollisionChecker::collisionCheckingHelper(std::vector<hmpl::State> *curve) {
    for (auto &state_itr : *curve) {
        // path collision checking in global frame
        // get the car footprint , prepare for the collision checking
        std::vector<hmpl::Circle> footprint =
            this->car_.getCurrentCenters(state_itr);
        // footprint checking
        for (auto &circle_itr : footprint) {
            grid_map::Position pos(circle_itr.position.x,
                                   circle_itr.position.y);
            if (this->in_gm_.maps.isInside(pos)) {
                double clearance = this->in_gm_.getObstacleDistance(pos);
                if (clearance < circle_itr.r) {  // collision
                    state_itr.v = 1;  // the v field is used as a collision
                    // checking flag
                } else {
                    state_itr.v = 0;
                }
            } else {
                state_itr.v = 1;
            }
        }
    }
}

}
