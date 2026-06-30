#pragma once

#include "constraint/Constraint.hpp"
#include "constraint/RevoluteJointConstraint.hpp"

namespace Constraint
{

class RevoluteJointVelocityMotorConstraint : public XPBDConstraint<1, 2, 0>
{
public:
    RevoluteJointVelocityMotorConstraint(const RevoluteJointConstraint& rev_constraint, Real initial_velocity);
    RevoluteJointVelocityMotorConstraint(const NormedRevoluteJointConstraint& rev_constraint, Real initial_velocity);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

    Real velocity() const { return _velocity; }
    void setVelocity(Real new_vel) { _velocity = new_vel; }

    void updateTarget(Real dt);

private:
    Mat3r _or1;
    Mat3r _or2;

    Real _target;
    Real _velocity;
};



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


// class OneSidedRevoluteJointLimitConstraint : public XPBDConstraint<1, 1, 0>
// {
// public:
//     OneSidedRevoluteJointLimitConstraint(const OneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);
//     OneSidedRevoluteJointLimitConstraint(const NormedOneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha=0);

//     OneSidedRevoluteJointLimitConstraint(const OneSidedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);
//     OneSidedRevoluteJointLimitConstraint(const NormedOneSidedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha=0);

//     virtual bool isInequality() const override { return true; }

//     virtual ConstraintVecType evaluate() const override;
//     virtual GradientMatType gradient() const override;

// protected:
//     Mat3r _or1;

//     Mat3r _base_or;

//     Real _min_angle;
//     Real _max_angle;
// };

} // namespace Constraint