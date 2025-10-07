#pragma once

#include "common/common.hpp"
#include "common/math.hpp"

namespace SimObject
{

/** Represents a general "particle" that has both position and orientation with some mass and rotational inertia.
 * This particle can be both a node in a discretized Cosserat rod, or a rigid body.
 * 
 * TODO: add fixed flag
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
    bool fixed=false;             // if true, the particle is "fixed" and should not move

    /** Updates the particle based on its current velocity (in the absence of constraints) and applied external wrench.
     * @param dt - the time step
     * @param F_ext - external force
     * @param T_ext - external torque
     */
    void inertialUpdate(Real dt, const Vec3r& F_ext, const Vec3r& T_ext)
    {
        if (fixed)
            return;

        // positional update
        position += dt*lin_velocity + dt*dt/mass*F_ext;

        // rotational update
        Vec3r Ib_inv = 1.0/Ib.array();
        Vec3r so3_update = dt*ang_velocity + dt*dt*Ib_inv.asDiagonal()*(T_ext - ang_velocity.cross(Ib.asDiagonal()*ang_velocity));
        orientation = Math::Plus_SO3(orientation, so3_update);
    }

    /** Updates the particle based on its current velocity (in the absence of constraints) with no applied external wrench.
     * @param dt - the time step
     */
    void inertialUpdate(Real dt)
    {
        inertialUpdate(dt, Vec3r::Zero(), Vec3r::Zero());
    }

    /** Updates the particle given some change in position and orientiation.
     * @param dpos - the change in position (specified in global coordinates)
     * @param dor - the change in orientation - a member of the Lie algebra so(3)
     */
    void positionUpdate(const Vec3r& dpos, const Vec3r& dor)
    {
        if (fixed)
            return;
            
        position += dpos;
        orientation = Math::Plus_SO3(orientation, dor);
    }

    /** Updates the particle given some change in position and orientation.
     * @param dp - a 6-vector with the combined change in position (first 3 coords) and orientation (last 3 coords).
     */
    void positionUpdate(const Vec6r& dp)
    {
        positionUpdate(dp.head<3>(), dp.tail<3>());
    }

    /** Updates the particle velocity given a time step, and previous position and orientation.
     * @param dt - the time step
     * @param prev_pos - the previous position of the particle (at the end of the last time step)
     * @param prev_or - the previous orientation of the particle (at the end of the last time step)
     */
    void velocityUpdate(Real dt, const Vec3r& prev_pos, const Mat3r& prev_or)
    {
        lin_velocity = (position - prev_pos)/dt;
        ang_velocity = Math::Minus_SO3(orientation, prev_or)/dt;
    }
};

} // namespace SimObject