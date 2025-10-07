#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class PointLineConstraint : public XPBDConstraint<1,3>
{

public:
    PointLineConstraint(SimObject::OrientedParticle* point, SimObject::OrientedParticle* endpoint1, SimObject::OrientedParticle* endpoint2);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool use_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache=false) const override;

};

} // namespace Constraint