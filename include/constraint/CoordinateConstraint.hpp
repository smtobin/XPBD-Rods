#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class OneSidedCoordinateConstraint : public XPBDConstraint<1, 1, 0>
{
public:
    OneSidedCoordinateConstraint(
        SimObject::OrientedParticle* particle1,
        const Vec3r& fixed_pos, int c_index
    );

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Vec3r _fixed_pos;

    int _c_index;

};

}