#pragma once

#include "constraint/Constraint.hpp"

#include "simobject/rod/RodElement.hpp"

namespace Constraint
{

template <int Order1, int Order2>
class RodRodCollisionConstraint : public XPBDConstraint<1, Order1+1 + Order2+1>
{
public:
    constexpr static int StateDim = XPBDConstraint<1, Order1+1 + Order2+1>::StateDim;
    constexpr static int ConstraintDim = XPBDConstraint<1, Order1+1 + Order2+1>::ConstraintDim;
    constexpr static int NumParticles = XPBDConstraint<1, Order1+1 + Order2+1>::NumParticles;
    using ConstraintVecType = typename XPBDConstraint<1, Order1+1 + Order2+1>::ConstraintVecType;
    using AlphaVecType = typename XPBDConstraint<1, Order1+1 + Order2+1>::AlphaVecType;
    using GradientMatType = typename XPBDConstraint<1, Order1+1 + Order2+1>::GradientMatType;
    using SingleParticleGradientMatType = typename XPBDConstraint<1, Order1+1 + Order2+1>::SingleParticleGradientMatType;

    RodRodCollisionConstraint(
        SimObject::RodElement<Order1>* element1,
        Real s_hat1, Vec3r cp_local1,
        SimObject::RodElement<Order2>* element2,
        Real s_hat2, Vec3r cp_local2,
        const Vec3r& n,
        Real mu_s, Real mu_d
    );

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

    void applyFriction(Real lambda_n) const;

private:
    using XPBDConstraint<1, Order1+1 + Order2+1>::_particles;

    /** Rod elements */
    SimObject::RodElement<Order1>* _element1;
    SimObject::RodElement<Order2>* _element2;

    /** Interpolation parameter for the rod segments in [0,1] */
    Real _s_hat1;
    Real _s_hat2;

    /** Contact points (expressed in local interpolated frame) for each rod segment */
    Vec3r _cp_local1;
    Vec3r _cp_local2;


    /** Collision normal.
     * Points outward from rod 1 towards the rod 2.
     */
    Vec3r _n;

    /** Friction coefficients for the collision */
    Real _mu_s;
    Real _mu_d;
};

} // namespace Constraint