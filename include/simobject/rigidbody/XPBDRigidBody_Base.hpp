#pragma once

#include "config/XPBDRigidBodyConfig.hpp"
#include "simobject/XPBDObject_Base.hpp"
#include "simobject/OrientedParticle.hpp"

namespace SimObject
{

class XPBDRigidBody_Base : public XPBDObject_Base
{

public:
    XPBDRigidBody_Base(const Config::XPBDRigidBodyConfig& config);

    const OrientedParticle& com() const { return _com; }
    OrientedParticle& com() { return _com; }

    /** Required by XPBDObject_Base */
    virtual std::vector<const OrientedParticle*> particles() const override { return {&_com}; }

    /** Sets the rigid body to be fixed or not. */
    virtual void setFixed(bool fixed) { _com.fixed = fixed; }


    /** Performs necessary setup to prepare the rod for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup()
    {
        // make sure that the mass and rotational inertia have been calculated
        assert(_com.mass != -1 && _com.Ib != Vec3r::Zero());
    }

    /** Updates the object in the absence of constraints. */
    virtual void inertialUpdate(Real dt) override
    {
        /** TODO: incorporate applied external forces/torques. For now, the only force or torque is gravity. */
        const Vec3r F_grav(0, -G_ACCEL * _com.mass, 0);
        _com.inertialUpdate(dt, F_grav, Vec3r::Zero());
    }

    /** Solves internal constraints for this object. These don't exist for a rigid body. */
    virtual void internalConstraintSolve(Real /* dt */) override {}

    /** Updates the rigid body's velocity. */
    virtual void velocityUpdate(Real dt) override
    {
        _com.velocityUpdate(dt, _prev_position, _prev_orientation);

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
