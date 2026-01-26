#include "constraint/PrismaticJointLimitConstraint.hpp"

namespace Constraint
{

PrismaticJointLimitConstraint::PrismaticJointLimitConstraint(const PrismaticJointConstraint& pr_constraint, Real min_dist, Real max_dist, Real alpha)
    : XPBDConstraint<1, 2>(pr_constraint.particles(), AlphaVecType(alpha)),
    _r1(pr_constraint.bodyJointOffset1()),
    _or1(pr_constraint.bodyJointOrientationOffset1()),
    _r2(pr_constraint.bodyJointOffset2()),
    _min_dist(min_dist), _max_dist(max_dist)
{

}

PrismaticJointLimitConstraint::PrismaticJointLimitConstraint(const NormedPrismaticJointConstraint& pr_constraint, Real min_dist, Real max_dist, Real alpha)
    : XPBDConstraint<1, 2>(pr_constraint.particles(), AlphaVecType(alpha)),
    _r1(pr_constraint.bodyJointOffset1()),
    _or1(pr_constraint.bodyJointOrientationOffset1()),
    _r2(pr_constraint.bodyJointOffset2()),
    _min_dist(min_dist), _max_dist(max_dist)
{

}

PrismaticJointLimitConstraint::ConstraintVecType PrismaticJointLimitConstraint::evaluate() const
{
    // position vector (global) between joint positions: (p2 - p1)
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _particles[1]->position + _particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    // joint 1 orientation
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    // rotate positional difference into joint 1 local frame
    const Vec3r joint_dp = joint_or1.transpose() * dp;

    // translation distance is 3rd component
    Real dist = joint_dp[2];

    // difference with min
    Real min_diff = dist - _min_dist;
    // difference with max
    Real max_diff = _max_dist - dist;

    ConstraintVecType C;
    C[0] = std::min(min_diff, max_diff);
    return C;
}

PrismaticJointLimitConstraint::GradientMatType PrismaticJointLimitConstraint::gradient(bool /* update_cache */) const
{
    GradientMatType grad;

    // position vector (global) between joint positions: (p2 - p1)
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _particles[1]->position + _particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    // joint 1 orientation
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    // rotate positional difference into joint 1 local frame
    const Vec3r joint_dp = joint_or1.transpose() * dp;

    // translation distance is 3rd component
    Real dist = joint_dp[2];

    // difference with min
    Real min_diff = dist - _min_dist;
    // difference with max
    Real max_diff = _max_dist - dist;

    // taken from PrismaticJointConstraint
    // the limit constraint is the 3rd row of each of these matrices
    const Mat3r dCp_dp1 = -joint_or1.transpose();
    const Mat3r dCp_dp2 = joint_or1.transpose();

    const Mat3r dCp_dor1 = joint_or1.transpose() * Math::Skew3(dp) * _particles[0]->orientation + _or1.transpose() * Math::Skew3(_r1);
    const Mat3r dCp_dor2 = -joint_or1.transpose() * _particles[1]->orientation * Math::Skew3(_r2);

    if (min_diff < max_diff)
    {
        // the minimum limit is active
        grad.block<1,3>(0,0) = dCp_dp1.row(2);
        grad.block<1,3>(0,3) = dCp_dor1.row(2);
        grad.block<1,3>(0,6) = dCp_dp2.row(2);
        grad.block<1,3>(0,9) = dCp_dor2.row(2);
    }
    else
    {
        // the maximum limit is active
        grad.block<1,3>(0,0) = -dCp_dp1.row(2);
        grad.block<1,3>(0,3) = -dCp_dor1.row(2);
        grad.block<1,3>(0,6) = -dCp_dp2.row(2);
        grad.block<1,3>(0,9) = -dCp_dor2.row(2);
    }

    return grad;
}

PrismaticJointLimitConstraint::SingleParticleGradientMatType PrismaticJointLimitConstraint::singleParticleGradient(const SimObject::OrientedParticle* /* node_ptr */, bool /* use_cache */) const
{
    throw std::runtime_error("PrismaticJointLimitConstraint has no singleParticleGradient");
    return SingleParticleGradientMatType::Zero();
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

OneSidedPrismaticJointLimitConstraint::OneSidedPrismaticJointLimitConstraint(
    const OneSidedPrismaticJointConstraint& pr_constraint, Real min_dist, Real max_dist, Real alpha)
    : XPBDConstraint<1, 1>(pr_constraint.particles(), AlphaVecType(alpha)),
    _r2(pr_constraint.bodyJointOffset2()),
    _base_pos(pr_constraint.basePosition()),
    _base_or(pr_constraint.baseOrientation()),
    _min_dist(min_dist), _max_dist(max_dist)
{

}

OneSidedPrismaticJointLimitConstraint::OneSidedPrismaticJointLimitConstraint(
    const NormedOneSidedPrismaticJointConstraint& pr_constraint, Real min_dist, Real max_dist, Real alpha)
    : XPBDConstraint<1, 1>(pr_constraint.particles(), AlphaVecType(alpha)),
    _r2(pr_constraint.bodyJointOffset2()),
    _base_pos(pr_constraint.basePosition()),
    _base_or(pr_constraint.baseOrientation()),
    _min_dist(min_dist), _max_dist(max_dist)
{

}

OneSidedPrismaticJointLimitConstraint::ConstraintVecType OneSidedPrismaticJointLimitConstraint::evaluate() const
{
    // position vector (global) between joint positions: (p2 - p1)
    const Vec3r joint_pos1 = _base_pos;
    const Vec3r joint_pos2 = _particles[0]->position + _particles[0]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    // joint 1 orientation
    const Mat3r joint_or1 = _base_or;
    // rotate positional difference into joint 1 local frame
    const Vec3r joint_dp = joint_or1.transpose() * dp;

    // translation distance is 3rd component
    Real dist = joint_dp[2];

    // difference with min
    Real min_diff = dist - _min_dist;
    // difference with max
    Real max_diff = _max_dist - dist;

    ConstraintVecType C;
    C[0] = std::min(min_diff, max_diff);
    return C;
}

OneSidedPrismaticJointLimitConstraint::GradientMatType OneSidedPrismaticJointLimitConstraint::gradient(bool /* update_cache */) const
{
    GradientMatType grad;

    // position vector (global) between joint positions: (p2 - p1)
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r2;
    const Vec3r joint_pos2 = _base_pos;
    const Vec3r dp = joint_pos2 - joint_pos1;

    // joint 1 orientation
    const Mat3r joint_or1 = _base_or;
    // rotate positional difference into joint 1 local frame
    const Vec3r joint_dp = joint_or1.transpose() * dp;

    // translation distance is 3rd component
    Real dist = joint_dp[2];

    // difference with min
    Real min_diff = dist - _min_dist;
    // difference with max
    Real max_diff = _max_dist - dist;

    // taken from PrismaticJointConstraint
    // the limit constraint is the 3rd row of each of these matrices
    const Mat3r dCp_dp2 = -joint_or1.transpose();
    const Mat3r dCp_dor2 = joint_or1.transpose() * _particles[0]->orientation * Math::Skew3(_r2);

    if (min_diff < max_diff)
    {
        // the minimum limit is active
        grad.block<1,3>(0,0) = dCp_dp2.row(2);
        grad.block<1,3>(0,3) = dCp_dor2.row(2);
    }
    else
    {
        // the maximum limit is active
        grad.block<1,3>(0,0) = -dCp_dp2.row(2);
        grad.block<1,3>(0,3) = -dCp_dor2.row(2);
    }

    return grad;
}

OneSidedPrismaticJointLimitConstraint::SingleParticleGradientMatType OneSidedPrismaticJointLimitConstraint::singleParticleGradient(const SimObject::OrientedParticle* /* node_ptr */, bool /* use_cache */) const
{
    throw std::runtime_error("OneSidedPrismaticJointLimitConstraint has no singleParticleGradient");
    return SingleParticleGradientMatType::Zero();
}

} // namespace Constraint