#pragma once

#include "constraint/Constraint.hpp"
#include "constraint/RevoluteJointConstraint.hpp"

namespace Constraint
{

class RevoluteJointLimitConstraint : public XPBDConstraint<1, 2, 0>
{
public:
    RevoluteJointLimitConstraint(const RevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);
    RevoluteJointLimitConstraint(const NormedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);

    // twist limit constraint for spherical joints
    RevoluteJointLimitConstraint(const SphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);
    RevoluteJointLimitConstraint(const NormedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Mat3r _or1;
    Mat3r _or2;

    Real _min_angle;
    Real _max_angle;
};



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


class OneSidedRevoluteJointLimitConstraint : public XPBDConstraint<1, 1, 0>
{
public:
    OneSidedRevoluteJointLimitConstraint(const OneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);
    OneSidedRevoluteJointLimitConstraint(const NormedOneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);

    OneSidedRevoluteJointLimitConstraint(const OneSidedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);
    OneSidedRevoluteJointLimitConstraint(const NormedOneSidedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Mat3r _or1;

    Mat3r _base_or;

    Real _min_angle;
    Real _max_angle;
};

} // namespace Constraint