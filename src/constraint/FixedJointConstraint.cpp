#include "constraint/FixedJointConstraint.hpp"
#include "common/math.hpp"

namespace Constraint
{

FixedJointConstraint::FixedJointConstraint(
    SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
    SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2,
    const AlphaVecType& alpha
)
    : XPBDConstraint<6,2>({particle1, particle2}, alpha),
    _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{
}

FixedJointConstraint::ConstraintVecType FixedJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _particles[1]->position + _particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);

    ConstraintVecType C_vec;
    C_vec.head<3>() = dp;
    C_vec.tail<3>() = dtheta;

    return C_vec;
}

FixedJointConstraint::GradientMatType FixedJointConstraint::gradient(bool update_cache) const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Mat3r dCp_dp1 = -Mat3r::Identity(); 
    const Mat3r dCp_dp2 = Mat3r::Identity();
    const Mat3r dCp_dor1 = _particles[0]->orientation * Math::Skew3(_r1);
    const Mat3r dCp_dor2 = -_particles[1]->orientation * Math::Skew3(_r2);

    // gradients of rotational constraints
    const Mat3r dCor_dp1 = Mat3r::Zero();
    const Mat3r dCor_dp2 = Mat3r::Zero();

    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);
    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    const Mat3r dCor_dor1 = -jac_inv.transpose() * _or1.transpose();
    const Mat3r dCor_dor2 = jac_inv * _or2.transpose();

    grad.block<3,3>(0,0) = dCp_dp1;
    grad.block<3,3>(0,3) = dCp_dor1;
    grad.block<3,3>(0,6) = dCp_dp2;
    grad.block<3,3>(0,9) = dCp_dor2;
    grad.block<3,3>(3,0) = dCor_dp1;
    grad.block<3,3>(3,3) = dCor_dor1;
    grad.block<3,3>(3,6) = dCor_dp2;
    grad.block<3,3>(3,9) = dCor_dor2;

    // update cache if specified to
    if (update_cache)
    {
        _cached_gradients[0] = grad.block<6,6>(0,0);
        _cached_gradients[1] = grad.block<6,6>(0,6);
    }
    

    return grad;
}

FixedJointConstraint::SingleParticleGradientMatType FixedJointConstraint::singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache) const
{
    if (use_cache)
    {
        if (node_ptr == _particles[0])
        {
            return _cached_gradients[0];
        }
        else if (node_ptr == _particles[1])
        {
            return _cached_gradients[1];
        }
    }

    if (node_ptr == _particles[0])
    {
        const Mat3r dCp_dp1 = -Mat3r::Identity(); 
        const Mat3r dCp_dor1 = _particles[0]->orientation * Math::Skew3(_r1);
        const Mat3r dCor_dp1 = Mat3r::Zero();

        const Mat3r joint_or1 = _particles[0]->orientation * _or1;
        const Mat3r joint_or2 = _particles[1]->orientation * _or2;
        const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);
        const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
        const Mat3r dCor_dor1 = -jac_inv.transpose() * _or1.transpose();

        SingleParticleGradientMatType grad;
        grad.block<3,3>(0,0) = dCp_dp1;
        grad.block<3,3>(0,3) = dCp_dor1;
        grad.block<3,3>(3,0) = dCor_dp1;
        grad.block<3,3>(3,3) = dCor_dor1;

        return grad;
    }
    else if (node_ptr == _particles[1])
    {
        const Mat3r dCp_dp2 = Mat3r::Identity();
        const Mat3r dCp_dor2 = -_particles[1]->orientation * Math::Skew3(_r2);
        const Mat3r dCor_dp2 = Mat3r::Zero();

        const Mat3r joint_or1 = _particles[0]->orientation * _or1;
        const Mat3r joint_or2 = _particles[1]->orientation * _or2;
        const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);
        const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
        const Mat3r dCor_dor2 = jac_inv * _or2.transpose();

        SingleParticleGradientMatType grad;
        grad.block<3,3>(0,0) = dCp_dp2;
        grad.block<3,3>(0,3) = dCp_dor2;
        grad.block<3,3>(3,0) = dCor_dp2;
        grad.block<3,3>(3,3) = dCor_dor2;

        return grad;
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


OneSidedFixedJointConstraint::OneSidedFixedJointConstraint(
    const Vec3r& ref_position, const Mat3r& ref_orientation,
    SimObject::OrientedParticle* particle, const Vec3r& r1, const Mat3r& or1,
    const AlphaVecType& alpha
)
    : XPBDConstraint<SimObject::OrientedParticle::DOF, 1>({particle}, alpha),
    _r1(r1), _or1(or1), _ref_position(ref_position), _ref_orientation(ref_orientation)
{
}

OneSidedFixedJointConstraint::ConstraintVecType OneSidedFixedJointConstraint::evaluate() const 
{
    // fixed base constraint
    ConstraintVecType C_vec;
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    C_vec.head<3>() = joint_pos1 - _ref_position;

    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    C_vec.tail<3>() = Math::Minus_SO3(joint_or1, _ref_orientation);
    return C_vec;
}

OneSidedFixedJointConstraint::GradientMatType OneSidedFixedJointConstraint::gradient(bool update_cache) const
{
    GradientMatType grad = GradientMatType::Zero();
    // fixed base constraint gradient
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Vec3r dtheta0 = Math::Minus_SO3(joint_or1, _ref_orientation);
    const Mat3r jac_inv0 = Math::ExpMap_InvRightJacobian(dtheta0);
    grad.block<3,3>(0,0) = Mat3r::Identity();
    grad.block<3,3>(3,3) = jac_inv0 * _or1.transpose();

    if (update_cache)
    {
        _cached_gradients[0] = grad;
    }

    return grad;
}

OneSidedFixedJointConstraint::SingleParticleGradientMatType OneSidedFixedJointConstraint::singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache) const
{
    if (particle_ptr == _particles[0])
    {
        if (use_cache)
            return _cached_gradients[0];
        
        SingleParticleGradientMatType grad = SingleParticleGradientMatType::Zero();
        // fixed base constraint gradient
        const Mat3r joint_or1 = _particles[0]->orientation * _or1;
        const Vec3r dtheta0 = Math::Minus_SO3(joint_or1, _ref_orientation);
        const Mat3r jac_inv0 = Math::ExpMap_InvRightJacobian(dtheta0);
        grad.block<3,3>(0,0) = Mat3r::Identity();
        grad.block<3,3>(3,3) = jac_inv0 * _or1.transpose();

        return grad;
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}

} // namespace Constraint