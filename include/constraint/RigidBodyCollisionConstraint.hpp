#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class RigidBodyCollisionConstraint : public XPBDConstraint<1, 2>
{
public:
    RigidBodyCollisionConstraint(
        SimObject::OrientedParticle* com1, const Vec3r& r1,
        SimObject::OrientedParticle* com2, const Vec3r& r2,
        const Vec3r& n
    );

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    /** Local offset to contact point in body 1 frame */
    Vec3r _r1;

    /** Local offset to contact point in body 2 frame */
    Vec3r _r2;

    /** Collision normal */
    Vec3r _n;
};

} // namespace Constraint