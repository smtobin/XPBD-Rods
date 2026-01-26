#pragma once

#include "constraint/Constraint.hpp"
#include "constraint/PrismaticJointConstraint.hpp"

namespace Constraint
{

class PrismaticJointLimitConstraint : public XPBDConstraint<1, 2>
{
public:
    PrismaticJointLimitConstraint(const PrismaticJointConstraint& rev_constraint, Real min_dist, Real max_dist, Real alpha=0);
    PrismaticJointLimitConstraint(const NormedPrismaticJointConstraint& rev_constraint, Real min_dist, Real max_dist, Real alpha=0);

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    Vec3r _r1;
    Mat3r _or1;

    Vec3r _r2;

    Real _min_dist;
    Real _max_dist;
};



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


class OneSidedPrismaticJointLimitConstraint : public XPBDConstraint<1, 1>
{
public:
    OneSidedPrismaticJointLimitConstraint(const OneSidedPrismaticJointConstraint& rev_constraint, Real min_dist, Real max_dist, Real alpha=0);
    OneSidedPrismaticJointLimitConstraint(const NormedOneSidedPrismaticJointConstraint& rev_constraint, Real min_dist, Real max_dist, Real alpha=0);

    virtual bool isInequality() const override { return true; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    Vec3r _r2;

    Vec3r _base_pos;
    Mat3r _base_or;

    Real _min_dist;
    Real _max_dist;
};

} // namespace Constraint