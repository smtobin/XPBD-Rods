#pragma once

#include "common/common.hpp"
#include "config/XPBDRigidBodyConfig.hpp"
#include "simobject/XPBDObject_Base.hpp"
#include "simobject/OrientedParticle.hpp"

namespace SimObject
{

class XPBDRigidBody_Base : public XPBDObject_Base
{

public:
    XPBDRigidBody_Base(const XPBDRigidBodyConfig& config)
        : XPBDObject_Base(config), _com()
    {
        _com.position = config.initialPosition();
        _com.orientation = Math::RotMatFromXYZEulerAngles(config.initialRotation());
        _com.lin_velocity = config.initialVelocity();
        _com.ang_velocity = config.initialAngularVelocity();

        _prev_position = _com.position;
        _prev_orientation = _com.orientation;

        // Derived classes are responsible for setting the mass and rotational inertia
        _com.mass = -1;
        _com.Ib = -1;
    }

    /** Performs necessary setup to prepare the rod for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup()
    {
        // make sure that the mass and rotational inertia have been calculated
        assert(_com.mass != -1 && _com.Ib != -1);
    }

    /** Updates the object in the absence of constraints. */
    virtual void inertialUpdate(Real dt) override
    {
        /** TODO: incorporate applied external forces/torques. For now, the only force or torque is gravity. */
        const Vec3r F_grav(0, 0, -G_ACCEL * _com.mass);
        _com.inertialUpdate(dt);
    }

    /** Solves internal constraints for this object. These don't exist for a rigid body. */
    virtual void internalConstraintSolve(Real dt) override {}

    /** Updates the rigid body's velocity. */
    virtual void velocityUpdate(Real dt) override
    {
        _com.velocityUpdate(dt, _prev_com.position, _prev_com.orientation);

        _prev_position = _com.position;
        _prev_orientation = _com.orientation;
    }

protected:

    /** The oriented particle that represents this rigid body - corresponds to a frame at the center of mass */
    OrientedParticle _com;

    /** Keep track of the previous state of the rigid body */
    Vec3r _prev_position;
    Mat3r _prev_orientation;

};


} // namespace SimObject
