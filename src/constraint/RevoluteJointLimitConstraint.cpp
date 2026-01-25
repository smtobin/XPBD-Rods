#include "constraint/RevoluteJointLimitConstraint.hpp"

namespace Constraint
{

RevoluteJointLimitConstraint::RevoluteJointLimitConstraint(const RevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 2>(rev_constraint.particles(), AlphaVecType(alpha)),
    _or1(rev_constraint.bodyJointOrientationOffset1()),
    _or2(rev_constraint.bodyJointOrientationOffset2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

RevoluteJointLimitConstraint::RevoluteJointLimitConstraint(const NormedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 2>(rev_constraint.particles(), AlphaVecType(alpha)),
    _or1(rev_constraint.bodyJointOrientationOffset1()),
    _or2(rev_constraint.bodyJointOrientationOffset2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

RevoluteJointLimitConstraint::ConstraintVecType RevoluteJointLimitConstraint::evaluate() const
{
    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // difference with min
    Real min_diff = dtheta[3] - _min_angle;
    // difference with max
    Real max_diff = _max_angle - dtheta[3];

    ConstraintVecType C;
    C[0] = std::min(min_diff, max_diff);
    return C;
}

RevoluteJointLimitConstraint::GradientMatType RevoluteJointLimitConstraint::gradient(bool /* update_cache */) const
{
    GradientMatType grad;

    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // difference with min
    Real min_diff = dtheta[3] - _min_angle;
    // difference with max
    Real max_diff = _max_angle - dtheta[3];

    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);

    Mat3r dC_dR1, dC_dR2;
    if (min_diff < max_diff)
    {
        // the minimum limit is active
        dC_dR1 = jac_inv * _or1.transpose();
        dC_dR2 = -jac_inv.transpose() * _or2.transpose();
    }
    else
    {
        // the maximum limit is active
        dC_dR1 = -jac_inv * _or1.transpose();
        dC_dR2 = jac_inv.transpose() * _or2.transpose();
    }
    
    grad.block<1,3>(0,0) = Vec3r::Zero();
    grad.block<1,3>(0,3) = dC_dR1.row(3);
    grad.block<1,3>(0,6) = Vec3r::Zero();
    grad.block<1,3>(0,9) = dC_dR2.row(3);

    return grad;
}

RevoluteJointLimitConstraint::SingleParticleGradientMatType RevoluteJointLimitConstraint::singleParticleGradient(const SimObject::OrientedParticle* /* node_ptr */, bool /* use_cache */) const
{
    throw std::runtime_error("RevoluteJointLimitConstraint has no singleParticleGradient");
    return SingleParticleGradientMatType::Zero();
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

OneSidedRevoluteJointLimitConstraint::OneSidedRevoluteJointLimitConstraint(const OneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 1>(rev_constraint.particles(), AlphaVecType(alpha)),
    _or1(rev_constraint.bodyJointOrientationOffset1()),
    _base_or(rev_constraint.jointOrientation2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

OneSidedRevoluteJointLimitConstraint::OneSidedRevoluteJointLimitConstraint(const NormedOneSidedRevoluteJointConstraint& rev_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 1>(rev_constraint.particles(), AlphaVecType(alpha)),
    _or1(rev_constraint.bodyJointOrientationOffset1()),
    _base_or(rev_constraint.jointOrientation2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

OneSidedRevoluteJointLimitConstraint::ConstraintVecType OneSidedRevoluteJointLimitConstraint::evaluate() const
{
    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _base_or;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // difference with min
    Real min_diff = dtheta[3] - _min_angle;
    // difference with max
    Real max_diff = _max_angle - dtheta[3];

    ConstraintVecType C;
    C[0] = std::min(min_diff, max_diff);
    return C;
}

OneSidedRevoluteJointLimitConstraint::GradientMatType OneSidedRevoluteJointLimitConstraint::gradient(bool /* update_cache */) const
{
    GradientMatType grad;

    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _base_or;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // difference with min
    Real min_diff = dtheta[3] - _min_angle;
    // difference with max
    Real max_diff = _max_angle - dtheta[3];

    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);

    Mat3r dC_dR1, dC_dR2;
    if (min_diff < max_diff)
    {
        // the minimum limit is active
        dC_dR1 = jac_inv * _or1.transpose();
    }
    else
    {
        // the maximum limit is active
        dC_dR1 = -jac_inv * _or1.transpose();
    }
    
    grad.block<1,3>(0,0) = Vec3r::Zero();
    grad.block<1,3>(0,3) = dC_dR1.row(3);

    return grad;
}

OneSidedRevoluteJointLimitConstraint::SingleParticleGradientMatType OneSidedRevoluteJointLimitConstraint::singleParticleGradient(const SimObject::OrientedParticle* /* node_ptr */, bool /* use_cache */) const
{
    throw std::runtime_error("OneSidedRevoluteJointLimitConstraint has no singleParticleGradient");
    return SingleParticleGradientMatType::Zero();
}

} // namespace Constraint