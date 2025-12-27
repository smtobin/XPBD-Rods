#pragma once

#include "common/common.hpp"
#include "config/XPBDObjectConfig.hpp"
#include "simobject/OrientedParticle.hpp"
#include "constraint/AllConstraints.hpp"

namespace SimObject
{

class XPBDObject_Base
{

public:
    XPBDObject_Base(const Config::XPBDObjectConfig& config)
        : _name(config.name())
    {}

    /** Performs necessary setup to prepare the object for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup() = 0;

    /** Updates the object in the absence of constraints. Should be called at the beginning of a time step. */
    virtual void inertialUpdate(Real dt) = 0;

    /** Solves constraints defined within the object. */
    virtual void internalConstraintSolve(Real dt) = 0;

    /** Updates the object's velocity. Should be called at the end of a time step. */
    virtual void velocityUpdate(Real dt) = 0;

    /** Returns the "internal" constraints associated with this object. */
    const XPBDConstraints_Container& internalConstraints() const { return _internal_constraints; }
    const VecXr& internalLambda() const { return _internal_lambda; }

    virtual std::vector<const OrientedParticle*> particles() const = 0;

protected:
    std::string _name;

    /** Stores the "internal" constraints for this object.
     * The internal constraints are constraints that are intentionally hidden from the Gauss-Seidel solver so that they can be solved for in a special way.
     * E.g. the rod elastic constraints - these should not be solved 1-at-a-time but all together for improved convergence
     */
    XPBDConstraints_Container _internal_constraints;

    /** Stores the Lagrange multipliers associated with the internal constraints for this object.
     * It is up to the derived classes to allocated an appropriate amount of space and update this during a solve.
     */
    VecXr _internal_lambda;

};

} // namespace SimObject