#pragma once

#include "constraint/Constraint.hpp"
#include "simobject/rod/RodElement.hpp"

namespace Constraint
{

template <int Order>
class RodRigidBodyCollisionConstraint : public XPBDConstraint<1, 1 + Order+1, 0>
{
public:
    using BaseConstraintType = XPBDConstraint<1, 1 + Order+1, 0>;
    constexpr static int StateDim = BaseConstraintType::StateDim;
    constexpr static int ConstraintDim = BaseConstraintType::ConstraintDim;
    constexpr static int NumOrientedParticles = BaseConstraintType::NumOrientedParticles;
    using ConstraintVecType = typename BaseConstraintType::ConstraintVecType;
    using AlphaVecType = typename BaseConstraintType::AlphaVecType;
    using GradientMatType = typename BaseConstraintType::GradientMatType;

    RodRigidBodyCollisionConstraint(
        SimObject::RodElement<Order>* element,
        Real s_hat, const Vec3r& cp_local_rod,
        SimObject::OrientedParticle* com_rb, const Vec3r& cp_local_rb,
        const Vec3r& n,
        Real mu_s, Real mu_d
    );

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

    void applyFriction(Real lambda_n) const;
    void applyRestitution() const;

private:
    using BaseConstraintType::_oriented_particles;

    /** Rod element in collision */
    SimObject::RodElement<Order>* _element;

    /** Interpolation parameter for the rod segment in [0,1] */
    Real _s_hat;

    /** Local offset to contact point in interpolated rod frame */
    Vec3r _cp_local_rod;

    /** Local offset to contact point in rigid body local frame */
    Vec3r _cp_local_rb;

    /** Collision normal.
     * Points outward from the rod towards the rigid body.
     */
    Vec3r _n;

    /** Friction coefficients for the collision. */
    Real _mu_s;
    Real _mu_d;
};



////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

template <int Order>
class OneSidedRodRigidBodyCollisionConstraint : public XPBDConstraint<1, Order+1, 0>
{
public:
    using BaseConstraintType = XPBDConstraint<1, Order+1, 0>;
    constexpr static int StateDim = BaseConstraintType::StateDim;
    constexpr static int ConstraintDim = BaseConstraintType::ConstraintDim;
    constexpr static int NumOrientedParticles = BaseConstraintType::NumOrientedParticles;
    using ConstraintVecType = typename BaseConstraintType::ConstraintVecType;
    using AlphaVecType = typename BaseConstraintType::AlphaVecType;
    using GradientMatType = typename BaseConstraintType::GradientMatType;

    OneSidedRodRigidBodyCollisionConstraint(
        SimObject::RodElement<Order>* element,
        Real s_hat, const Vec3r& cp_local_rod,
        const Vec3r& rb_cp,
        const Vec3r& n,
        Real mu_s, Real mu_d
    );

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

    void applyFriction(Real lambda_n) const;
    void applyRestitution() const;

private:
    using BaseConstraintType::_oriented_particles;
    
    /** Rod element in collision */
    SimObject::RodElement<Order>* _element;

    /** Interpolation parameter for the rod segment in [0,1] */
    Real _s_hat;

    /** Local offset to contact point in interpolated rod frame */
    Vec3r _cp_local_rod;

    /** Contact point (global frame) */
    Vec3r _cp;

    /** Collision normal.
     * Points outward from the rod towards the rigid body.
     */
    Vec3r _n;

    /** Friction coefficients for the collision. */
    Real _mu_s;
    Real _mu_d;
};

} // namespace Constraint