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

///////////////////////////////////////////////////////////////////////////////

class OneSidedPointLineConstraint : public XPBDConstraint<1, 1, 0>
{

public:
    OneSidedPointLineConstraint(SimObject::OrientedParticle* point, const Vec3r& endpoint1, const Vec3r& endpoint2);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    // constant endpoints that define the line
    Vec3r _endpoint1;
    Vec3r _endpoint2;
};

} // namespace Constraint