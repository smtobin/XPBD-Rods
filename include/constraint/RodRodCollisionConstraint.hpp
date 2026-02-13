#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class RodRodCollisionConstraint : public XPBDConstraint<1, 4>
{
public:
    RodRodCollisionConstraint(
        SimObject::XPBDRod* rod1,
        SimObject::OrientedParticle* p1_rod1, SimObject::OrientedParticle* p2_rod1,
        Real beta1, Real r_rod1,
        SimObject::XPBDRod* rod2,
        SimObject::OrientedParticle* p1_rod2, SimObject::OrientedParticle* p2_rod2,
        Real beta2, Real r_rod2,
        const Vec3r& n
    );

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    /** Pointers to rods in collision */
    SimObject::XPBDRod* _rod1;
    SimObject::XPBDRod* _rod2;

    /** Interpolation parameter for the rod segments in [0,1] */
    Real _beta1;
    Real _beta2;

    /** Radius of rods (assuming circular cross-section for now) */
    Real _r_rod1;
    Real _r_rod2;


    /** Collision normal.
     * Points outward from rod 1 towards the rod 2.
     */
    Vec3r _n;
};

} // namespace Constraint