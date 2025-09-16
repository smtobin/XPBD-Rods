#pragma once

#include "common/common.hpp"
#include "simobject/XPBDObject_Base.hpp"
#include "simobject/OrientedParticle.hpp"

namespace SimObject
{

class XPBDRigidBody_Base : public XPBDObject_Base
{

public:
    XPBDRigidBody_Base(const std::string& name);

    /** Performs necessary setup to prepare the rod for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup();

    /** Updates the object in the absence of constraints. */
    virtual void inertialUpdate(Real dt);

    /** Solves internal constraints for this object. These don't exist for a rigid body. */
    virtual void internalConstraintSolve(Real dt) {}

    /** Updates the rigid body's velocity. */
    virtual void velocityUpdate();

protected:

    /** The oriented particle that represents this rigid body - corresponds to a frame at the center of mass */
    OrientedParticle _com;

};


} // namespace SimObject
