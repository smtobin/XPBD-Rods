#include "constraint/PrismaticJointConstraint.hpp"

namespace Constraint
{

PrismaticJointConstraint::PrismaticJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<5,2,0>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

PrismaticJointConstraint::ConstraintVecType PrismaticJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Minus_SO3(joint_or1, joint_or2);
    const Vec3r joint_dp = joint_or1.transpose() * dp;
    
    ConstraintVecType C_vec;
    C_vec.head<2>() = joint_dp.head<2>();
    C_vec.tail<3>() = dor;

    return C_vec;
}

PrismaticJointConstraint::GradientMatType PrismaticJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    const Mat3r dCp_dp1 = -joint_or1.transpose();
    const Mat3r dCp_dp2 = joint_or1.transpose();

    const Mat3r dCp_dor1 = joint_or1.transpose() * Math::Skew3(dp) * _oriented_particles[0]->orientation + _or1.transpose() * Math::Skew3(_r1);
    const Mat3r dCp_dor2 = -joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_r2);
    
    const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);
    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    const Mat3r dCor_dor1 = jac_inv * _or1.transpose();
    const Mat3r dCor_dor2 = -jac_inv.transpose() * _or2.transpose();

    grad.block<2,3>(0,0) = dCp_dp1.block<2,3>(0,0);
    grad.block<2,3>(0,3) = dCp_dor1.block<2,3>(0,0);
    grad.block<2,3>(0,6) = dCp_dp2.block<2,3>(0,0);
    grad.block<2,3>(0,9) = dCp_dor2.block<2,3>(0,0);
    grad.block<3,3>(2,0) = Mat3r::Zero();
    grad.block<3,3>(2,3) = dCor_dor1;
    grad.block<3,3>(2,6) = Mat3r::Zero();
    grad.block<3,3>(2,9) = dCor_dor2;
    

    return grad;

}

Real PrismaticJointConstraint::evaluateSingle(int index) const
{
    if (index < 2)
    {
        const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
        const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
        const Vec3r dp = joint_pos2 - joint_pos1;

        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Vec3r joint_dp = joint_or1.transpose() * dp;
        return joint_dp[index];
    }
    else
    {
        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
        const Vec3r dor = Math::Minus_SO3(joint_or1, joint_or2);
        return dor[index-2];
    }
}

Eigen::Matrix<Real, 1, PrismaticJointConstraint::StateDim> PrismaticJointConstraint::gradientSingle(int index) const
{
    Eigen::Matrix<Real, 1, StateDim> grad_row;
    if (index < 2)
    {
        // gradients of positional constraints
        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
        const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
        const Vec3r dp = joint_pos2 - joint_pos1;

        grad_row.block<1,3>(0,0) = -joint_or1.transpose().row(index);
        const Mat3r dCp_dor1 = joint_or1.transpose() * Math::Skew3(dp) * _oriented_particles[0]->orientation + _or1.transpose() * Math::Skew3(_r1);
        grad_row.block<1,3>(0,3) = dCp_dor1.row(index);

        grad_row.block<1,3>(0,6) = joint_or1.transpose().row(index);
        const Mat3r dCp_dor2 = -joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_r2);
        grad_row.block<1,3>(0,9) = dCp_dor2.row(index);
    }
    else
    {
        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
        const Vec3r dtheta = Math::Minus_SO3(joint_or1, joint_or2);
        const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
        const Mat3r dCor_dor1 = jac_inv * _or1.transpose();
        const Mat3r dCor_dor2 = -jac_inv.transpose() * _or2.transpose();

        grad_row.block<1,3>(0,0) = Vec3r::Zero();
        grad_row.block<1,3>(0,3) = dCor_dor1.row(index-2);
        grad_row.block<1,3>(0,6) = Vec3r::Zero();
        grad_row.block<1,3>(0,9) = dCor_dor2.row(index-2);
    }

    return grad_row;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

NormedPrismaticJointConstraint::NormedPrismaticJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<2,2,0>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

NormedPrismaticJointConstraint::ConstraintVecType NormedPrismaticJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Minus_SO3(joint_or1, joint_or2);
    const Vec3r joint_dp = joint_or1.transpose() * dp;

    ConstraintVecType C_vec;
    C_vec[0] = joint_dp.head<2>().norm();
    C_vec[1] = dor.norm();

    return C_vec;
}

NormedPrismaticJointConstraint::GradientMatType NormedPrismaticJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
    const Vec3r dp_full = joint_pos2 - joint_pos1;

    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Minus_SO3(joint_or1, joint_or2);
    const Vec3r joint_dp_full = joint_or1.transpose() * dp_full;
    const Vec2r joint_dp = joint_dp_full.head<2>();

    Vec3r dCp_dp1, dCp_dp2, dCp_dor1, dCp_dor2;
    if (joint_dp.norm() < CONSTRAINT_EPS)
    {
        dCp_dp1 = Vec3r(1,0,0);
        dCp_dp2 = Vec3r(1,0,0);
        dCp_dor1 = Vec3r(1,0,0);
        dCp_dor2 = Vec3r(1,0,0);
    }
    else
    {
        const Vec2r n = joint_dp/joint_dp.norm();
        const Vec3r n3(n[0], n[1], 0);
        dCp_dp1 = -n3.transpose()*joint_or1.transpose(); 
        dCp_dp2 = n3.transpose()*joint_or1.transpose();

        dCp_dor1 = n3.transpose() * (joint_or1.transpose() * Math::Skew3(dp_full) * _oriented_particles[0]->orientation + _or1.transpose() * Math::Skew3(_r1));
        dCp_dor2 = -n3.transpose() * joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_r2);
    }

    // gradients of rotational constraints
    Vec3r dCor_dor1, dCor_dor2;
    if (dor.norm() < CONSTRAINT_EPS)
    {
        dCor_dor1 = Vec3r(1,0,0);
        dCor_dor2 = Vec3r(1,0,0);
    }
    else
    {
        const Vec3r n = dor/dor.norm();
        dCor_dor1 = n.transpose() * _or1.transpose();
        dCor_dor2 = -n.transpose() * _or2.transpose();
    }
    
    
    grad.block<1,3>(0,0) = dCp_dp1;
    grad.block<1,3>(0,3) = dCp_dor1;
    grad.block<1,3>(0,6) = dCp_dp2;
    grad.block<1,3>(0,9) = dCp_dor2;
    grad.block<1,3>(1,0) = Vec3r::Zero();
    grad.block<1,3>(1,3) = dCor_dor1;
    grad.block<1,3>(1,6) = Vec3r::Zero();
    grad.block<1,3>(1,9) = dCor_dor2;
    

    return grad;

}

Real NormedPrismaticJointConstraint::evaluateSingle(int index) const
{
    if (index == 0)
    {
        const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
        const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
        const Vec3r dp = joint_pos2 - joint_pos1;

        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Vec3r joint_dp = joint_or1.transpose() * dp;
        return joint_dp.head<2>().norm();
    }
    else
    {
        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
        const Vec3r dor = Math::Minus_SO3(joint_or1, joint_or2);
        return dor.norm();
    }
}

Eigen::Matrix<Real, 1, NormedPrismaticJointConstraint::StateDim> NormedPrismaticJointConstraint::gradientSingle(int index) const
{
    Eigen::Matrix<Real, 1, StateDim> grad_row;
    if (index == 0)
    {
        // gradients of positional constraints
        const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
        const Vec3r joint_pos2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;
        const Vec3r dp_full = joint_pos2 - joint_pos1;

        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Vec3r joint_dp_full = joint_or1.transpose() * dp_full;
        const Vec2r joint_dp = joint_dp_full.head<2>();

        if (joint_dp.norm() < CONSTRAINT_EPS)
        {
            grad_row.block<1,3>(0,0) = Vec3r(1,0,0);
            grad_row.block<1,3>(0,3) = Vec3r(1,0,0);
            grad_row.block<1,3>(0,6) = Vec3r(1,0,0);
            grad_row.block<1,3>(0,9) = Vec3r(1,0,0);
        }
        else
        {
            const Vec2r n = joint_dp/joint_dp.norm();
            const Vec3r n3(n[0], n[1], 0);
            grad_row.block<1,3>(0,0) = -n3.transpose()*joint_or1.transpose(); 
            grad_row.block<1,3>(0,6) = n3.transpose()*joint_or1.transpose();

            grad_row.block<1,3>(0,3) = n3.transpose() * (joint_or1.transpose() * Math::Skew3(dp_full) * _oriented_particles[0]->orientation + _or1.transpose() * Math::Skew3(_r1));
            grad_row.block<1,3>(0,9) = -n3.transpose() * joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_r2);
        }
    }
    else
    {   
        // gradients of rotational constraints
        const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
        const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
        const Vec3r dor = Math::Minus_SO3(joint_or1, joint_or2);

        grad_row.block<1,3>(0,0) = Vec3r::Zero();
        grad_row.block<1,3>(0,6) = Vec3r::Zero();
        if (dor.norm() < CONSTRAINT_EPS)
        {
            grad_row.block<1,3>(0,3) = Vec3r(1,0,0);
            grad_row.block<1,3>(0,9) = Vec3r(1,0,0);
        }
        else
        {
            const Vec3r n = dor/dor.norm();
            grad_row.block<1,3>(0,3) = n.transpose() * _or1.transpose();
            grad_row.block<1,3>(0,9) = -n.transpose() * _or2.transpose();
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

OneSidedPrismaticJointConstraint::OneSidedPrismaticJointConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<5,1,0>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r2(joint_pos), _or2(joint_or)
{

}

OneSidedPrismaticJointConstraint::ConstraintVecType OneSidedPrismaticJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r2;
    const Vec3r dp = joint_pos1 - _base_pos;

    const Mat3r joint_or2 = _oriented_particles[0]->orientation * _or2;
    const Vec3r dor = Math::Minus_SO3(joint_or2, _base_or);
    const Vec3r joint_dp = _base_or.transpose() * dp;
    
    ConstraintVecType C_vec;
    C_vec.head<2>() = joint_dp.head<2>();
    C_vec.tail<3>() = dor;

    return C_vec;
}

OneSidedPrismaticJointConstraint::GradientMatType OneSidedPrismaticJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Mat3r joint_or2 = _oriented_particles[0]->orientation * _or2;

    const Mat3r dCp_dp1 = _base_or.transpose();
    const Mat3r dCp_dor1 = -_base_or.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r2);
    
    const Vec3r dtheta = Math::Minus_SO3(joint_or2, _base_or);
    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    const Mat3r dCor_dor1 = jac_inv * _or2.transpose();

    grad.block<2,3>(0,0) = dCp_dp1.block<2,3>(0,0);
    grad.block<2,3>(0,3) = dCp_dor1.block<2,3>(0,0);
    grad.block<3,3>(2,0) = Mat3r::Zero();
    grad.block<3,3>(2,3) = dCor_dor1;
    

    return grad;

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NormedOneSidedPrismaticJointConstraint::NormedOneSidedPrismaticJointConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<2,1,0>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r2(joint_pos), _or2(joint_or)
{

}

NormedOneSidedPrismaticJointConstraint::ConstraintVecType NormedOneSidedPrismaticJointConstraint::evaluate() const
{
    const Vec3r joint_pos2 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r2;
    const Vec3r dp = joint_pos2 - _base_pos;

    const Mat3r joint_or2 = _oriented_particles[0]->orientation * _or2;
    const Vec3r dor = Math::Minus_SO3(joint_or2, _base_or);
    const Vec3r joint_dp = _base_or.transpose() * dp;

    ConstraintVecType C_vec;
    C_vec[0] = joint_dp.head<2>().norm();
    C_vec[1] = dor.norm();

    return C_vec;
}

NormedOneSidedPrismaticJointConstraint::GradientMatType NormedOneSidedPrismaticJointConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Vec3r joint_pos2 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r2;
    const Vec3r dp_full = joint_pos2 - _base_pos;

    const Mat3r joint_or2 = _oriented_particles[0]->orientation * _or2;
    const Vec3r dor = Math::Minus_SO3(joint_or2, _base_or);
    const Vec3r joint_dp_full = _base_or.transpose() * dp_full;
    const Vec2r joint_dp = joint_dp_full.head<2>();

    Vec3r dCp_dp1, dCp_dor1;
    if (joint_dp.norm() < CONSTRAINT_EPS)
    {
        dCp_dp1 = Vec3r(1,0,0);
        dCp_dor1 = Vec3r(1,0,0);
    }
    else
    {
        const Vec2r n = joint_dp/joint_dp.norm();
        const Vec3r n3(n[0], n[1], 0);
        dCp_dp1 = n3.transpose()*_base_or.transpose();
        dCp_dor1 = -n3.transpose() * _base_or.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r2);
    }

    // gradients of rotational constraints
    Vec3r dCor_dor1, dCor_dor2;
    if (dor.norm() < CONSTRAINT_EPS)
    {
        dCor_dor1 = Vec3r(1,0,0);
    }
    else
    {
        const Vec3r n = dor/dor.norm();
        dCor_dor1 = n.transpose() * _or2.transpose();
    }
    
    
    grad.block<1,3>(0,0) = dCp_dp1;
    grad.block<1,3>(0,3) = dCp_dor1;
    grad.block<1,3>(1,0) = Vec3r::Zero();
    grad.block<1,3>(1,3) = dCor_dor1;
    

    return grad;

}

} // namespace Constraint