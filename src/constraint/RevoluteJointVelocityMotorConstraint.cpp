#include "constraint/RevoluteJointVelocityMotorConstraint.hpp"

namespace Constraint
{

RevoluteJointVelocityMotorConstraint::RevoluteJointVelocityMotorConstraint(const RevoluteJointConstraint& rev_constraint, Real initial_velocity)
    : XPBDConstraint<1, 2, 0>(rev_constraint.orientedParticles(), AlphaVecType::Zero()),
    _or1(rev_constraint.bodyJointOrientationOffset1()),
    _or2(rev_constraint.bodyJointOrientationOffset2()),
    _velocity(initial_velocity)
{
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);
    _target = dtheta[2];
}

RevoluteJointVelocityMotorConstraint::RevoluteJointVelocityMotorConstraint(const NormedRevoluteJointConstraint& rev_constraint, Real initial_velocity)
    : XPBDConstraint<1, 2, 0>(rev_constraint.orientedParticles(), AlphaVecType::Zero()),
    _or1(rev_constraint.bodyJointOrientationOffset1()),
    _or2(rev_constraint.bodyJointOrientationOffset2()),
    _velocity(initial_velocity)
{
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);
    _target = dtheta[2];
}

void RevoluteJointVelocityMotorConstraint::updateTarget(Real dt)
{
    _target += dt*_velocity;

    // wrap target so that it is in [-180, 180] deg
    while (_target > M_PI)
        _target -= M_PI;
    
    while (_target < -M_PI)
        _target += M_PI;
}

RevoluteJointVelocityMotorConstraint::ConstraintVecType RevoluteJointVelocityMotorConstraint::evaluate() const
{
    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // be careful to "bridge the gap" between 180 and -180 deg
    // e.g. if target is -179 and the current dtheta[2] is +179, set target to be 181
    Real corr_target = _target;
    if (dtheta[2] > 3*M_PI/4 && _target < -3*M_PI/4)
        corr_target = _target + M_PI;
    if (dtheta[2] < -3*M_PI/4 && _target > 3*M_PI/4)
        corr_target = _target - M_PI;

    // difference from target
    Real diff = dtheta[2] - corr_target;

    ConstraintVecType C;
    C[0] = diff;
    return C;
}

RevoluteJointVelocityMotorConstraint::GradientMatType RevoluteJointVelocityMotorConstraint::gradient() const
{
    GradientMatType grad;

    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);

    Mat3r dC_dR1, dC_dR2;
    // the minimum limit is active
    dC_dR1 = jac_inv * _or1.transpose();
    dC_dR2 = -jac_inv.transpose() * _or2.transpose();
    
    grad.block<1,3>(0,0) = Vec3r::Zero();
    grad.block<1,3>(0,3) = dC_dR1.row(2);
    grad.block<1,3>(0,6) = Vec3r::Zero();
    grad.block<1,3>(0,9) = dC_dR2.row(2);

    return grad;
}

} // namespace Constraint