#pragma once

#include "constraint/Constraint.hpp"
#include "simobject/rod/XPBDRodSegment.hpp"

namespace Constraint
{

class RodRodCollisionConstraint : public XPBDConstraint<1, 4>
{
public:
    RodRodCollisionConstraint(
        SimObject::XPBDRodSegment segment1,     // copy segment by value, it is lightweight
        Real beta1, Vec3r cp_local1,
        SimObject::XPBDRodSegment segment2,
        Real beta2, Vec3r cp_local2,
        const Vec3r& n
    );

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

    void applyFriction(Real lambda_n, Real mu_s, Real mu_d) const {}

private:
    /** Pointers to rods in collision */
    SimObject::XPBDRodSegment _segment1;
    SimObject::XPBDRodSegment _segment2;

    /** Interpolation parameter for the rod segments in [0,1] */
    Real _beta1;
    Real _beta2;

    /** Contact points (expressed in local interpolated frame) for each rod segment */
    Vec3r _cp_local1;
    Vec3r _cp_local2;


    /** Collision normal.
     * Points outward from rod 1 towards the rod 2.
     */
    Vec3r _n;
};

} // namespace Constraint