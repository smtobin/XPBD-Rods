#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class RodRigidBodyCollisionConstraint : public XPBDConstraint<1, 3>
{
public:
    RodRigidBodyCollisionConstraint(
        SimObject::OrientedParticle* p1, SimObject::OrientedParticle* p2,
        Real beta, Real r_rod,
        SimObject::OrientedParticle* com_rb, const Vec3r& r_rb,
        const Vec3r& n
    );

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    /** Interpolation parameter for the rod segment in [0,1] */
    Real _beta;

    /** Radius of rod (assuming circular cross-section for now) */
    Real _r_rod;

    /** Local offset to contact point in rigid body local frame */
    Vec3r _r_rb;

    /** Collision normal.
     * Points outward from the rod towards the rigid body.
     */
    Vec3r _n;
};



////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

class OneSidedRodRigidBodyCollisionConstraint : public XPBDConstraint<1, 2>
{
public:
    OneSidedRodRigidBodyCollisionConstraint(
        SimObject::OrientedParticle* p1, SimObject::OrientedParticle* p2,
        Real beta, Real r_rod,
        const Vec3r& rb_cp,
        const Vec3r& n
    );

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    /** Interpolation parameter for the rod segment in [0,1] */
    Real _beta;

    /** Radius of rod (assuming circular cross-section for now) */
    Real _r_rod;

    /** Contact point (global frame) */
    Vec3r _cp;

    /** Collision normal.
     * Points outward from the rod towards the rigid body.
     */
    Vec3r _n;
};

} // namespace Constraint