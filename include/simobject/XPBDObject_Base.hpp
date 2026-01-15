#pragma once

#include "common/common.hpp"
// #include "common/constraint_containers.hpp"
#include "config/XPBDObjectConfig.hpp"
#include "simobject/OrientedParticle.hpp"

#include <memory>

namespace SimObject
{

struct ConstraintAndLambda
{
    XPBDConstraints_ConstPtrVariantType constraint;
    const Real* lambda;
    ConstraintAndLambda(const XPBDConstraints_ConstPtrVariantType& constraint_, const Real* lambda_)
        : constraint(constraint_), lambda(lambda_)
    {}
};

class XPBDObject_Base
{

public:
    XPBDObject_Base(const Config::XPBDObjectConfig& config);

    virtual ~XPBDObject_Base();

    // Move operations
    XPBDObject_Base(XPBDObject_Base&&) noexcept;
    XPBDObject_Base& operator=(XPBDObject_Base&&) noexcept;
    
    // Delete copy operations
    XPBDObject_Base(const XPBDObject_Base&) = delete;
    XPBDObject_Base& operator=(const XPBDObject_Base&) = delete;
    
    const std::string& name() const { return _name; }

    /** Performs necessary setup to prepare the object for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup() = 0;

    /** Updates the object in the absence of constraints. Should be called at the beginning of a time step. */
    virtual void inertialUpdate(Real dt) = 0;

    /** Solves constraints defined within the object. */
    virtual void internalConstraintSolve(Real dt) = 0;

    /** Updates the object's velocity. Should be called at the end of a time step. */
    virtual void velocityUpdate(Real dt) = 0;

    /** Returns the "internal" constraints associated with this object. */
    // const XPBDConstraints_Container& internalConstraints() const { return _internal_constraints; }
    // const VecXr& internalLambda() const { return _internalLambda(); }
    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const { return std::vector<ConstraintAndLambda>(); }

    virtual std::vector<const OrientedParticle*> particles() const = 0;

protected:
    std::string _name;

    /** Stores the "internal" constraints for this object.
     * The internal constraints are constraints that are intentionally hidden from the Gauss-Seidel solver so that they can be solved for in a special way.
     * E.g. the rod elastic constraints - these should not be solved 1-at-a-time but all together for improved convergence
     */
    XPBDConstraints_Container& _internalConstraints();
    const XPBDConstraints_Container& _internalConstraints() const;
    
    /** Stores the Lagrange multipliers associated with the internal constraints for this object.
     * It is up to the derived classes to allocated an appropriate amount of space and update this during a solve.
     */
    VecXr& _internalLambda();
    const VecXr& _internalLambda() const;

private:
    /** Use the PIMPL idiom to reduce build time. The internal constraints data structure is pretty heavy template, so try
     * and move it to the .cpp file so that every file that include XPBDObject_Base doesn't have to build the template.
     * 
     * The Impl struct has the following members:
     * - XPBDConstraints_Container _internal_constraints;
     * - VecXr _internalLambda();
     */
    struct Impl;
    std::unique_ptr<Impl> _impl;

    
    // XPBDConstraints_Container _internal_constraints;

    
    // VecXr _internalLambda();

};

} // namespace SimObject