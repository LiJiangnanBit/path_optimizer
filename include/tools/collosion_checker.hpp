//
// Created by yangt on 19-5-8.
//

#ifndef MPC_STATE_SAMPLING_COLLOSION_CHECKER_HPP
#define MPC_STATE_SAMPLING_COLLOSION_CHECKER_HPP

#include <car_model/car_geometry.hpp>
#include "tools/Map.hpp"

namespace PathOptimizationNS {

class CollisionChecker {
 public:
    CollisionChecker(const grid_map::GridMap &in_gm);
    /**
     * Path in global frame.  Collsion checking using footprint of the car.
     * Fill the collision flag to v field of State.  0 : collision,  1:
     * collision-free. Use 0 for collision because the initial value is 0. The
     * collision checking function will update the flag.
     * @param curve Path in ego frame
     * @return true if collision free, false otherwise. This funciton also modify
     * the v field of the state on the path. 0: collision, 1: collision-free
     */
    bool isSinglePathCollisionFree(std::vector<hmpl::State> *curve);

    /**
     * Path in global frame.  Collsion checking using bounding box first, if
     * collision occures, then check collisions using footprint circles
     * Fill the collision flag to v field of State.  0 : collision,  1:
     * collision-free
     * @param curve Path in the ego frame
     * @return true if the path is collision-free, false otherwise
     */
    bool isSinglePathCollisionFreeImproved(std::vector<hmpl::State> *curve);

    /**
     * Collsion checking using bounding box first, if
     * collision occures, then check collisions using footprint circles
     * @param current Vehicle state in global frame
     * @return true if collision free, false otherwise
     */
    bool isSingleStateCollisionFreeImproved(const hmpl::State &current);

    /**
     * Footprint circles collision Checking based on a single global state
     * @param current Current vehicle state in global frame
     * @return true if collision-free, false otherwise
     */
    bool isSingleStateCollisionFree(const hmpl::State &current);

    /**
     * This funciton is not used by internal collision checking algorithms, it's
     * for global path global collision checking.
     * @param curve Path in global frame, not ego frame
     */
    void collisionCheckingHelper(std::vector<hmpl::State> *curve);

 private:
    const Map in_gm_;
    hmpl::CarGeometry car_;
};

}

#endif //STATE_SAMPLING_COLLOSION_CHECKER_HPP
