#pragma once

#include "constraint/Constraint.hpp"
#include "constraint/RevoluteJointConstraint.hpp"

namespace Constraint
{

class RevoluteJointLimitConstraint : public XPBDConstraint<1, 2>
{
public:
    RevoluteJointLimitConstraint(const RevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);
    RevoluteJointLimitConstraint(const NormedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    Mat3r _or1;
    Mat3r _or2;

    Real _min_angle;
    Real _max_angle;
};



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


class OneSidedRevoluteJointLimitConstraint : public XPBDConstraint<1, 1>
{
public:
    OneSidedRevoluteJointLimitConstraint(const OneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);
    OneSidedRevoluteJointLimitConstraint(const NormedOneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    Mat3r _or1;

    Mat3r _base_or;

    Real _min_angle;
    Real _max_angle;
};

} // namespace Constraint