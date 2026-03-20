#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class PointLineConstraint : public XPBDConstraint<1, 3, 0>
{

public:
    PointLineConstraint(SimObject::OrientedParticle* point, SimObject::OrientedParticle* endpoint1, SimObject::OrientedParticle* endpoint2);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

};

} // namespace Constraint