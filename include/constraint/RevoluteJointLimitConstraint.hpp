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

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    const Vec3r& bodyJointOffset1() const { return _r1; }
    const Vec3r& bodyJointOffset2() const { return _r2; }

    const Mat3r& bodyJointOrientationOffset1() const { return _or1; }
    const Mat3r& bodyJointOrientationOffset2() const { return _or2; }

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    Vec3r _r1;
    Mat3r _or1;
    
    Vec3r _r2;
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