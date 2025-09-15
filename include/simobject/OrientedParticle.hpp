#pragma once

#include "common/common.hpp"

namespace SimObject
{

/** Represents a general "particle" that has both position and orientation with some mass and rotational inertia.
 * This particle can be both a node in a discretized Cosserat rod, or a rigid body.
 */
struct OrientedParticle
{
    constexpr static int DOF = 6;
    Vec3r position;         // position (global frame) of the particle
    Vec3r lin_velocity;     // translational velocity (global frame) of the particle
    Mat3r orientation;      // orientation of the particle (w.r.t. global frame)
    Vec3r ang_velocity;     // body angular velocity of the particle
    Real mass;              // mass of the particle
    Vec3r Ib;               // body rotational inertia of the particle (diagonal, so represented by a 3-vector)
};

} // namespace SimObject