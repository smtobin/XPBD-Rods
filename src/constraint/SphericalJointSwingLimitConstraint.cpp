#include "constraint/SphericalJointSwingLimitConstraint.hpp"

namespace Constraint
{

SphericalJointSwingLimitConstraint::SphericalJointSwingLimitConstraint(const SphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 2>(sph_constraint.particles(), AlphaVecType(alpha)),
    _or1(sph_constraint.bodyJointOrientationOffset1()),
    _or2(sph_constraint.bodyJointOrientationOffset2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

SphericalJointSwingLimitConstraint::SphericalJointSwingLimitConstraint(const NormedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 2>(sph_constraint.particles(), AlphaVecType(alpha)),
    _or1(sph_constraint.bodyJointOrientationOffset1()),
    _or2(sph_constraint.bodyJointOrientationOffset2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

SphericalJointSwingLimitConstraint::ConstraintVecType SphericalJointSwingLimitConstraint::evaluate() const
{
    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // swing angle = norm of first 2 components
    Real swing_angle = dtheta.head<2>().norm();

    // difference with min
    Real min_diff = swing_angle - _min_angle;
    // difference with max
    Real max_diff = _max_angle - swing_angle;

    ConstraintVecType C;
    C[0] = std::min(min_diff, max_diff);
    return C;
}

SphericalJointSwingLimitConstraint::GradientMatType SphericalJointSwingLimitConstraint::gradient(bool /* update_cache */) const
{
    GradientMatType grad;

    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // swing angle = norm of first 2 components
    Real swing_angle = dtheta.head<2>().norm();

    // difference with min
    Real min_diff = swing_angle - _min_angle;
    // difference with max
    Real max_diff = _max_angle - swing_angle;

    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    Vec2r n_swing = dtheta.head<2>() / swing_angle;

    Vec3r dC_dR1, dC_dR2;
    // constraint gradient is not defined when the swing angle = 0
    // either constraint shouldn't be active when swing angle = 0 anyways
    if (swing_angle < CONSTRAINT_EPS)
    {
        // arbitrary gradient (C = 0 so it doesn't really matter)
        dC_dR1 = Vec3r(1,1,0);
        dC_dR2 = Vec3r(-1,-1,0);
    }
    else
    {
        if (min_diff < max_diff)
        {
            // the minimum limit is active
            dC_dR1 = n_swing.transpose() * (jac_inv * _or1.transpose()).block<2,3>(0,0);
            dC_dR2 = -n_swing.transpose() * (jac_inv.transpose() * _or2.transpose()).block<2,3>(0,0);
        }
        else
        {
            // the maximum limit is active
            dC_dR1 = -n_swing.transpose() * (jac_inv * _or1.transpose()).block<2,3>(0,0);
            dC_dR2 = n_swing.transpose() * (jac_inv.transpose() * _or2.transpose()).block<2,3>(0,0);
        }
    }
    
    grad.block<1,3>(0,0) = Vec3r::Zero();
    grad.block<1,3>(0,3) = dC_dR1;
    grad.block<1,3>(0,6) = Vec3r::Zero();
    grad.block<1,3>(0,9) = dC_dR2;

    return grad;
}

SphericalJointSwingLimitConstraint::SingleParticleGradientMatType SphericalJointSwingLimitConstraint::singleParticleGradient(const SimObject::OrientedParticle* /* node_ptr */, bool /* use_cache */) const
{
    throw std::runtime_error("SphericalJointSwingLimitConstraint has no singleParticleGradient");
    return SingleParticleGradientMatType::Zero();
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

OneSidedSphericalJointSwingLimitConstraint::OneSidedSphericalJointSwingLimitConstraint(
    const OneSidedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 1>(sph_constraint.particles(), AlphaVecType(alpha)),
    _or1(sph_constraint.bodyJointOrientationOffset1()),
    _base_or(sph_constraint.jointOrientation2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

OneSidedSphericalJointSwingLimitConstraint::OneSidedSphericalJointSwingLimitConstraint(
    const NormedOneSidedSphericalJointConstraint& sph_constraint, Real min_angle, Real max_angle, Real alpha)
    : XPBDConstraint<1, 1>(sph_constraint.particles(), AlphaVecType(alpha)),
    _or1(sph_constraint.bodyJointOrientationOffset1()),
    _base_or(sph_constraint.jointOrientation2()),
    _min_angle(min_angle), _max_angle(max_angle)
{

}

OneSidedSphericalJointSwingLimitConstraint::ConstraintVecType OneSidedSphericalJointSwingLimitConstraint::evaluate() const
{
    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _base_or;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // swing angle = norm of first 2 components
    Real swing_angle = dtheta.head<2>().norm();

    std::cout << "Theta: " << dtheta.transpose() << std::endl;
    std::cout << "Swing angle: " << swing_angle << std::endl;

    // difference with min
    Real min_diff = swing_angle - _min_angle;
    // difference with max
    Real max_diff = _max_angle - swing_angle;

    ConstraintVecType C;
    C[0] = std::min(min_diff, max_diff);
    return C;
}

OneSidedSphericalJointSwingLimitConstraint::GradientMatType OneSidedSphericalJointSwingLimitConstraint::gradient(bool /* update_cache */) const
{
    GradientMatType grad;

    // rotation vector between joint orientations: R1 - R2
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _base_or;
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);

    // swing angle = norm of first 2 components
    Real swing_angle = dtheta.head<2>().norm();

    // difference with min
    Real min_diff = swing_angle - _min_angle;
    // difference with max
    Real max_diff = _max_angle - swing_angle;

    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    Vec2r n_swing = dtheta.head<2>() / swing_angle;

    Vec3r dC_dR1;
    // constraint gradient is not defined when the swing angle = 0
    // either constraint shouldn't be active when swing angle = 0 anyways
    if (swing_angle < CONSTRAINT_EPS)
    {
        // arbitrary gradient (C = 0 so it doesn't really matter)
        dC_dR1 = Vec3r(1,1,0);
    }
    else
    {
        if (min_diff < max_diff)
        {
            // the minimum limit is active
            dC_dR1 = n_swing.transpose() * (jac_inv * _or1.transpose()).block<2,3>(0,0);
        }
        else
        {
            // the maximum limit is active
            dC_dR1 = -n_swing.transpose() * (jac_inv * _or1.transpose()).block<2,3>(0,0);
        }
    }

    
    grad.block<1,3>(0,0) = Vec3r::Zero();
    grad.block<1,3>(0,3) = dC_dR1;

    return grad;
}

OneSidedSphericalJointSwingLimitConstraint::SingleParticleGradientMatType OneSidedSphericalJointSwingLimitConstraint::singleParticleGradient(const SimObject::OrientedParticle* /* node_ptr */, bool /* use_cache */) const
{
    throw std::runtime_error("OneSidedSphericalJointSwingLimitConstraint has no singleParticleGradient");
    return SingleParticleGradientMatType::Zero();
}

} // namespace Constraint