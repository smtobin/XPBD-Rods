#include "constraint/SphericalJointConstraint.hpp"

namespace Constraint
{

SphericalJointConstraint::SphericalJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<3,2,0>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

SphericalJointConstraint::ConstraintVecType SphericalJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos1 - joint_pos2;
    
    ConstraintVecType C_vec = dp;

    return C_vec;
}

SphericalJointConstraint::GradientMatType SphericalJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Mat3r dCp_dp1 = Mat3r::Identity(); 
    const Mat3r dCp_dp2 = -Mat3r::Identity();
    const Mat3r dCp_dor1 = -_oriented_particles[0]->orientation * Math::Skew3(_r1);
    const Mat3r dCp_dor2 = _oriented_particles[1]->orientation * Math::Skew3(_r2);

    grad.block<3,3>(0,0) = dCp_dp1;
    grad.block<3,3>(0,3) = dCp_dor1;
    grad.block<3,3>(0,6) = dCp_dp2;
    grad.block<3,3>(0,9) = dCp_dor2;
    

    return grad;

}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

NormedSphericalJointConstraint::NormedSphericalJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<1,2,0>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

NormedSphericalJointConstraint::ConstraintVecType NormedSphericalJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos1 - joint_pos2;

    ConstraintVecType C_vec;
    C_vec[0] = dp.norm();

    return C_vec;
}

NormedSphericalJointConstraint::GradientMatType NormedSphericalJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos1 - joint_pos2;

    Vec3r dCp_dp1, dCp_dp2, dCp_dor1, dCp_dor2;
    if (dp.norm() < CONSTRAINT_EPS)
    {
        dCp_dp1 = Vec3r(1,0,0);
        dCp_dp2 = Vec3r(1,0,0);
        dCp_dor1 = Vec3r(1,0,0);
        dCp_dor2 = Vec3r(1,0,0);
    }
    else
    {
        dCp_dp1 = dp/dp.norm(); 
        dCp_dp2 = -dp/dp.norm();
        dCp_dor1 = -dp.transpose()/dp.norm() * _oriented_particles[0]->orientation * Math::Skew3(_r1);
        dCp_dor2 = dp.transpose()/dp.norm() * _oriented_particles[1]->orientation * Math::Skew3(_r2);
    }
    
    grad.block<1,3>(0,0) = dCp_dp1;
    grad.block<1,3>(0,3) = dCp_dor1;
    grad.block<1,3>(0,6) = dCp_dp2;
    grad.block<1,3>(0,9) = dCp_dor2;
    

    return grad;

}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

OneSidedSphericalJointConstraint::OneSidedSphericalJointConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<3,1,0>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r1(joint_pos), _or1(joint_or)
{

}

OneSidedSphericalJointConstraint::ConstraintVecType OneSidedSphericalJointConstraint::evaluate() const
{
    const Vec3r joint_pos = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r dp = joint_pos - _base_pos;

    ConstraintVecType C_vec = dp;

    return C_vec;
}

OneSidedSphericalJointConstraint::GradientMatType OneSidedSphericalJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Mat3r dCp_dp = Mat3r::Identity();
    const Mat3r dCp_dor = -_oriented_particles[0]->orientation * Math::Skew3(_r1);

    grad.block<3,3>(0,0) = dCp_dp;
    grad.block<3,3>(0,3) = dCp_dor;
    

    return grad;

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NormedOneSidedSphericalJointConstraint::NormedOneSidedSphericalJointConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<1,1,0>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r1(joint_pos), _or1(joint_or)
{

}

NormedOneSidedSphericalJointConstraint::ConstraintVecType NormedOneSidedSphericalJointConstraint::evaluate() const
{
    const Vec3r joint_pos = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r dp = joint_pos - _base_pos;

    ConstraintVecType C_vec;
    C_vec[0] = dp.norm();

    return C_vec;
}

NormedOneSidedSphericalJointConstraint::GradientMatType NormedOneSidedSphericalJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Vec3r joint_pos = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r dp = joint_pos - _base_pos;

    Vec3r dCp_dp, dCp_dor;
    if (dp.norm() < CONSTRAINT_EPS)
    {
        dCp_dp = Vec3r(1,0,0);
        dCp_dor = Vec3r(1,0,0);
    }
    else
    {
        dCp_dp = dp / dp.norm();
        dCp_dor = -dp.transpose()/dp.norm() * _oriented_particles[0]->orientation * Math::Skew3(_r1);
    }

    grad.block<1,3>(0,0) = dCp_dp;
    grad.block<1,3>(0,3) = dCp_dor;
    

    return grad;

}


} // namespace Constraint