#pragma once

#include "common/common.hpp"

namespace SimObject
{

class XPBDObject_Base
{

public:
    XPBDObject_Base(const std::string& name)
        : _name(name)
    {}

    /** Performs necessary setup to prepare the object for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup() = 0;

    /** Updates the object in the absence of constraints. Should be called at the beginning of a time step. */
    virtual void inertialUpdate(Real dt) = 0;

    /** Solves constraints defined within the object. */
    virtual void internalConstraintSolve(Real dt) = 0;

    /** Updates the object's velocity. Should be called at the end of a time step. */
    virtual void velocityUpdate(Real dt) = 0;

protected:
    std::string _name;

};

} // namespace SimObject