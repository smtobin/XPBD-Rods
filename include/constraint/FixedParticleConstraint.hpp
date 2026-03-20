#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class OneSidedFixedParticleConstraint : public XPBDConstraint<3, 0, 1>
{
public:
    OneSidedFixedParticleConstraint(
        const Vec3r& ref_position,
        SimObject::Particle* particle, const Vec3r& r1,
        const AlphaVecType& alpha = AlphaVecType::Zero()
    );

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Vec3r _ref_position;
    Vec3r _r1;
};

} // namespace Constraint