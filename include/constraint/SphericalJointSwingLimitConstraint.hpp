#pragma once

#include "constraint/Constraint.hpp"
#include "constraint/SphericalJointConstraint.hpp"

namespace Constraint
{

class SphericalJointSwingLimitConstraint : public XPBDConstraint<1, 2>
{
public:
    SphericalJointSwingLimitConstraint(const SphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);
    SphericalJointSwingLimitConstraint(const NormedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);

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


class OneSidedSphericalJointSwingLimitConstraint : public XPBDConstraint<1, 1>
{
public:
    OneSidedSphericalJointSwingLimitConstraint(const OneSidedSphericalJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);
    OneSidedSphericalJointSwingLimitConstraint(const NormedOneSidedSphericalJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);

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