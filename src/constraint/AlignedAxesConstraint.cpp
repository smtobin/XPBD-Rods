#include "constraint/AlignedAxesConstraint.hpp"

#include <Eigen/Geometry>

namespace Constraint
{

AlignedAxesConstraint::AlignedAxesConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<2,2,0>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

AlignedAxesConstraint::ConstraintVecType AlignedAxesConstraint::evaluate() const
{
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);

    ConstraintVecType C_vec;
    C_vec.tail<2>() = dor.head<2>();

    return C_vec;
}

AlignedAxesConstraint::GradientMatType AlignedAxesConstraint::gradient() const
{
    GradientMatType grad;
    const Mat3r dCor_dp1 = Mat3r::Zero();
    const Mat3r dCor_dp2 = Mat3r::Zero();

    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Mat3r dCor_dor1 = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _oriented_particles[0]->orientation;
    const Mat3r dCor_dor2 = -Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_or2.col(2));
    grad.block<2,3>(0,0) = dCor_dp1.block<2,3>(0,0);
    grad.block<2,3>(0,3) = dCor_dor1.block<2,3>(0,0);
    grad.block<2,3>(0,6) = dCor_dp2.block<2,3>(0,0);
    grad.block<2,3>(0,9) = dCor_dor2.block<2,3>(0,0);
    

    return grad;
}

Real AlignedAxesConstraint::evaluateSingle(int index) const
{
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);
    return dor[index];
}

Eigen::Matrix<Real, 1, AlignedAxesConstraint::StateDim> AlignedAxesConstraint::gradientSingle(int index) const
{
    Eigen::Matrix<Real, 1, AlignedAxesConstraint::StateDim> grad_row;
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Mat3r dCor_dor1 = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _oriented_particles[0]->orientation;
    const Mat3r dCor_dor2 = -Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_or2.col(2));

    grad_row.block<1,3>(0,0) = Vec3r::Zero();
    grad_row.block<1,3>(0,3) = dCor_dor1.row(index);
    grad_row.block<1,3>(0,6) = Vec3r::Zero();
    grad_row.block<1,3>(0,9) = dCor_dor2.row(index);

    return grad_row;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

NormedAlignedAxesConstraint::NormedAlignedAxesConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<1,2,0>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

NormedAlignedAxesConstraint::ConstraintVecType NormedAlignedAxesConstraint::evaluate() const
{
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);

    ConstraintVecType C_vec;
    C_vec[0] = dor.norm();

    return C_vec;
}

NormedAlignedAxesConstraint::GradientMatType NormedAlignedAxesConstraint::gradient() const
{
    GradientMatType grad;

    // gradients of rotational constraints
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);

    const Vec3r dCor_dp1 = Vec3r::Zero();
    const Vec3r dCor_dp2 = Vec3r::Zero();

    Vec3r dCor_dor1, dCor_dor2;
    if (dor.norm() < CONSTRAINT_EPS)
    {
        dCor_dor1 = Vec3r(1,0,0);
        dCor_dor2 = Vec3r(1,0,0);
    }
    else
    {
        dCor_dor1 = dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _oriented_particles[0]->orientation;
        dCor_dor2 = -dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_or2.col(2));
    }
    grad.block<1,3>(0,0) = dCor_dp1;
    grad.block<1,3>(0,3) = dCor_dor1;
    grad.block<1,3>(0,6) = dCor_dp2;
    grad.block<1,3>(0,9) = dCor_dor2;
    

    return grad;

}

Real NormedAlignedAxesConstraint::evaluateSingle(int index) const
{
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);
    return dor.norm();
}

Eigen::Matrix<Real, 1, NormedAlignedAxesConstraint::StateDim> NormedAlignedAxesConstraint::gradientSingle(int index) const
{
    Eigen::Matrix<Real, 1, StateDim> grad_row;
    // gradients of rotational constraints
    const Mat3r joint_or1 = _oriented_particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _oriented_particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);

    const Vec3r dCor_dp1 = Vec3r::Zero();
    const Vec3r dCor_dp2 = Vec3r::Zero();

    grad_row.block<1,3>(0,0) = Vec3r::Zero();
    grad_row.block<1,3>(0,6) = Vec3r::Zero();

    if (dor.norm() < CONSTRAINT_EPS)
    {
        grad_row.block<1,3>(0,3) = Vec3r(1,0,0);
        grad_row.block<1,3>(0,9) = Vec3r(1,0,0);
    }
    else
    {
        grad_row.block<1,3>(0,3) = dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _oriented_particles[0]->orientation;
        grad_row.block<1,3>(0,9) = -dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_or2.col(2));
    }

    return grad_row;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

OneSidedAlignedAxesConstraint::OneSidedAlignedAxesConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<2,1,0>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r1(joint_pos), _or1(joint_or)
{

}

OneSidedAlignedAxesConstraint::ConstraintVecType OneSidedAlignedAxesConstraint::evaluate() const
{
    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);

    ConstraintVecType C_vec;
    C_vec = dor.head<2>();

    return C_vec;
}

OneSidedAlignedAxesConstraint::GradientMatType OneSidedAlignedAxesConstraint::gradient() const
{
    GradientMatType grad;

    // gradients of rotational constraints
    const Mat3r dCor_dp = Mat3r::Zero();

    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Mat3r dCor_dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * Math::Skew3(_base_or.col(2)) * _oriented_particles[0]->orientation;

    grad.block<2,3>(0,0) = dCor_dp.block<2,3>(0,0);
    grad.block<2,3>(0,3) = dCor_dor.block<2,3>(0,0);
    

    return grad;

}

Real OneSidedAlignedAxesConstraint::evaluateSingle(int index) const
{
    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);
    return dor[index];
}

Eigen::Matrix<Real, 1, OneSidedAlignedAxesConstraint::StateDim> OneSidedAlignedAxesConstraint::gradientSingle(int index) const
{
    Eigen::Matrix<Real, 1, StateDim> grad_row;

    // gradients of rotational constraints
    grad_row.block<1,3>(0,0) = Vec3r::Zero();

    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Mat3r dCor_dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * Math::Skew3(_base_or.col(2)) * _oriented_particles[0]->orientation;
    grad_row.block<1,3>(0,3) = dCor_dor.row(index);
    
    return grad_row;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NormedOneSidedAlignedAxesConstraint::NormedOneSidedAlignedAxesConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<1,1,0>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r1(joint_pos), _or1(joint_or)
{

}

NormedOneSidedAlignedAxesConstraint::ConstraintVecType NormedOneSidedAlignedAxesConstraint::evaluate() const
{
    const Vec3r joint_pos = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    const Vec3r dp = _base_pos - joint_pos;

    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);

    ConstraintVecType C_vec;
    C_vec[0] = dor.norm();

    return C_vec;
}

NormedOneSidedAlignedAxesConstraint::GradientMatType NormedOneSidedAlignedAxesConstraint::gradient() const
{
    GradientMatType grad;

    // gradients of rotational constraints
    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);
    
    Vec3r dCor_dor;
    if (dor.norm() < CONSTRAINT_EPS)
    {
        dCor_dor = Vec3r(1,0,0);
    }
    else
    {
        dCor_dor = dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * Math::Skew3(_base_or.col(2)) * _oriented_particles[0]->orientation;
    }

    grad.block<1,3>(0,0) = Vec3r::Zero();
    grad.block<1,3>(0,3) = dCor_dor;
    

    return grad;

}

Real NormedOneSidedAlignedAxesConstraint::evaluateSingle(int index) const
{
    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);
    return dor.norm();
    
}

Eigen::Matrix<Real, 1, NormedOneSidedAlignedAxesConstraint::StateDim> NormedOneSidedAlignedAxesConstraint::gradientSingle(int index) const
{
    Eigen::Matrix<Real, 1, StateDim> grad_row;

    // gradient of rotational constraint
    const Mat3r joint_or = _oriented_particles[0]->orientation * _or1;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);
    
    grad_row.block<1,3>(0,0) = Vec3r::Zero();

    Vec3r dCor_dor;
    if (dor.norm() < CONSTRAINT_EPS)
    {
        grad_row.block<1,3>(0,3) = Vec3r(1,0,0);
    }
    else
    {
        grad_row.block<1,3>(0,3) = dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * Math::Skew3(_base_or.col(2)) * _oriented_particles[0]->orientation;
    }
    

    return grad_row;
}

} // namespace Constraint