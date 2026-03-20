#pragma once

#include "common/common.hpp"
#include "common/math.hpp"

namespace SimObject
{

/** Represents a general "particle" that has 3 DOF in a vector space (e.g. a particle with only positional DOF).
 * This particle can be a node in a volumteric continuum, or a purely positional node in a rod,
 *  or even some other expression of generalized coordinates.
 * 
 */
struct Particle
{
    constexpr static int DOF = 3;
    Vec3r position;         // position (global frame) of the particle
    Vec3r velocity;     // translational velocity (global frame) of the particle
    Vec3r mass;              // mass of the particle
    Vec3r prev_position;    // previous position (at the end of the last time step) of the particle
    bool fixed=false;             // if true, the particle is "fixed" and should not move

    /** Updates the particle based on its current velocity (in the absence of constraints) and applied external wrench.
     * @param dt - the time step
     * @param F_ext - external force
     */
    void inertialUpdate(Real dt, const Vec3r& F_ext);

    /** Updates the particle based on its current velocity (in the absence of constraints) with no applied external wrench.
     * @param dt - the time step
     */
    void inertialUpdate(Real dt);

    /** Updates the particle given some change in position and orientiation.
     * @param dpos - the change in position (specified in global coordinates)
     */
    void positionUpdate(const Vec3r& dpos);

    /** Updates the particle given some change in position and orientation.
     * @param dp - a 6-vector with the combined change in position (first 3 coords) and orientation (last 3 coords).
     */
    void positionUpdate(const Vec6r& dp);

    /** Updates the particle velocity given a time step, and previous position and orientation.
     * @param dt - the time step
     */
    void velocityUpdate(Real dt);
};

} // namespace SimObject