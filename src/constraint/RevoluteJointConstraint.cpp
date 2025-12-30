#include "constraint/RevoluteJointConstraint.hpp"

namespace Constraint
{

RevoluteJointConstraint::RevoluteJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<5,2>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

RevoluteJointConstraint::ConstraintVecType RevoluteJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _particles[1]->position + _particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    // const Vec3r joint_axis1 = _particles[0]->orientation * _or1.col(2);
    // const Vec3r joint_axis2 = _particles[1]->orientation * _or2.col(2);
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);
    
    // const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);

    ConstraintVecType C_vec;
    C_vec.head<3>() = dp;
    C_vec.tail<2>() = dor.head<2>();

    return C_vec;
}

RevoluteJointConstraint::GradientMatType RevoluteJointConstraint::gradient(bool update_cache) const
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
    // const Vec3r dtheta = Math::Log_SO3(joint_or1.transpose() * joint_or2);
    // const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    // const Mat3r dCor_dor1 = -jac_inv.transpose() * _or1.transpose();   /** TODO: check this! */
    // const Mat3r dCor_dor2 = jac_inv * _or2.transpose();
    // const Vec3r joint_axis1 = _particles[0]->orientation * _or1 * Vec3r(0,0,1);
    // const Vec3r joint_axis2 = _particles[1]->orientation * _or2 * Vec3r(0,0,1);
    // const Mat3r dCor_dor1 = Math::Skew3(joint_axis2) * _particles[0]->orientation * Math::Skew3(_or1.col(2));
    const Mat3r dCor_dor1 = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _particles[0]->orientation;
    // const Mat3r dCor_dor2 = -Math::Skew3(joint_axis1) * _particles[1]->orientation * Math::Skew3(_or2.col(2));
    const Mat3r dCor_dor2 = -Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _particles[1]->orientation * Math::Skew3(_or2.col(2));

    grad.block<3,3>(0,0) = dCp_dp1;
    grad.block<3,3>(0,3) = dCp_dor1;
    grad.block<3,3>(0,6) = dCp_dp2;
    grad.block<3,3>(0,9) = dCp_dor2;
    grad.block<2,3>(3,0) = dCor_dp1.block<2,3>(0,0);
    grad.block<2,3>(3,3) = dCor_dor1.block<2,3>(0,0);
    grad.block<2,3>(3,6) = dCor_dp2.block<2,3>(0,0);
    grad.block<2,3>(3,9) = dCor_dor2.block<2,3>(0,0);

    // update cache if specified to
    if (update_cache)
    {
        _cached_gradients[0] = grad.block<5,6>(0,0);
        _cached_gradients[1] = grad.block<5,6>(0,6);
    }
    

    return grad;

}

RevoluteJointConstraint::SingleParticleGradientMatType RevoluteJointConstraint::singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache) const
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
        // const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);
        // const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
        // const Mat3r dCor_dor1 = -jac_inv.transpose() * _or1.transpose();
        // const Vec3r joint_axis1 = _particles[0]->orientation * _or1.col(2);
        // const Vec3r joint_axis2 = _particles[1]->orientation * _or2.col(2);
        // const Mat3r dCor_dor1 = Math::Skew3(joint_axis2) * _particles[0]->orientation * Math::Skew3(_or1.col(2));
        const Mat3r dCor_dor1 = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _particles[0]->orientation;


        SingleParticleGradientMatType grad;
        grad.block<3,3>(0,0) = dCp_dp1;
        grad.block<3,3>(0,3) = dCp_dor1;
        grad.block<2,3>(3,0) = dCor_dp1.block<2,3>(0,0);
        grad.block<2,3>(3,3) = dCor_dor1.block<2,3>(0,0);

        return grad;
    }
    else if (node_ptr == _particles[1])
    {
        const Mat3r dCp_dp2 = Mat3r::Identity();
        const Mat3r dCp_dor2 = -_particles[1]->orientation * Math::Skew3(_r2);
        const Mat3r dCor_dp2 = Mat3r::Zero();

        const Mat3r joint_or1 = _particles[0]->orientation * _or1;
        const Mat3r joint_or2 = _particles[1]->orientation * _or2;
        // const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);
        // const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
        // const Mat3r dCor_dor2 = jac_inv * _or2.transpose();

        // const Vec3r joint_axis1 = _particles[0]->orientation * _or1.col(2);
        // const Vec3r joint_axis2 = _particles[1]->orientation * _or2.col(2);
        // const Mat3r dCor_dor2 = -Math::Skew3(joint_axis1) * _particles[1]->orientation * Math::Skew3(_or2.col(2));
        const Mat3r dCor_dor2 = -Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _particles[1]->orientation * Math::Skew3(_or2.col(2));


        SingleParticleGradientMatType grad;
        grad.block<3,3>(0,0) = dCp_dp2;
        grad.block<3,3>(0,3) = dCp_dor2;
        grad.block<2,3>(3,0) = dCor_dp2.block<2,3>(0,0);
        grad.block<2,3>(3,3) = dCor_dor2.block<2,3>(0,0);

        return grad;
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

NormedRevoluteJointConstraint::NormedRevoluteJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2)
    : XPBDConstraint<2,2>({particle1, particle2}, AlphaVecType::Zero()), _r1(r1), _or1(or1), _r2(r2), _or2(or2)
{

}

NormedRevoluteJointConstraint::ConstraintVecType NormedRevoluteJointConstraint::evaluate() const
{
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _particles[1]->position + _particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    // const Vec3r joint_axis1 = _particles[0]->orientation * _or1.col(2);
    // const Vec3r joint_axis2 = _particles[1]->orientation * _or2.col(2);
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);
    
    // const Vec3r dtheta = Math::Minus_SO3(joint_or2, joint_or1);

    ConstraintVecType C_vec;
    C_vec[0] = dp.norm();
    C_vec[1] = dor.norm();

    return C_vec;
}

NormedRevoluteJointConstraint::GradientMatType NormedRevoluteJointConstraint::gradient(bool update_cache) const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _particles[1]->position + _particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

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
        dCp_dp1 = -dp/dp.norm(); 
        dCp_dp2 = dp/dp.norm();
        dCp_dor1 = dp.transpose()/dp.norm() * _particles[0]->orientation * Math::Skew3(_r1);
        dCp_dor2 = -dp.transpose()/dp.norm() * _particles[1]->orientation * Math::Skew3(_r2);
    }

    

    // gradients of rotational constraints
    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
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
        dCor_dor1 = dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _particles[0]->orientation;
        dCor_dor2 = -dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _particles[1]->orientation * Math::Skew3(_or2.col(2));
    }

    
    grad.block<1,3>(0,0) = dCp_dp1;
    grad.block<1,3>(0,3) = dCp_dor1;
    grad.block<1,3>(0,6) = dCp_dp2;
    grad.block<1,3>(0,9) = dCp_dor2;
    grad.block<1,3>(1,0) = dCor_dp1;
    grad.block<1,3>(1,3) = dCor_dor1;
    grad.block<1,3>(1,6) = dCor_dp2;
    grad.block<1,3>(1,9) = dCor_dor2;

    // update cache if specified to
    if (update_cache)
    {
        _cached_gradients[0] = grad.block<ConstraintDim,6>(0,0);
        _cached_gradients[1] = grad.block<ConstraintDim,6>(0,6);
    }
    

    return grad;

}

NormedRevoluteJointConstraint::SingleParticleGradientMatType NormedRevoluteJointConstraint::singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache) const
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

    const Vec3r joint_pos1 = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r joint_pos2 = _particles[1]->position + _particles[1]->orientation * _r2;
    const Vec3r dp = joint_pos2 - joint_pos1;

    const Mat3r joint_or1 = _particles[0]->orientation * _or1;
    const Mat3r joint_or2 = _particles[1]->orientation * _or2;
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * joint_or2.col(2);

    if (node_ptr == _particles[0])
    {
        Vec3r dCp_dp1, dCp_dor1;
        if (dp.norm() < CONSTRAINT_EPS)
        {
            dCp_dp1 = Vec3r(1,0,0);
            dCp_dor1 = Vec3r(1,0,0);
        }
        else
        {
            dCp_dp1 = -dp/dp.norm(); 
            dCp_dor1 = dp.transpose()/dp.norm() * _particles[0]->orientation * Math::Skew3(_r1);
        }

        const Vec3r dCor_dp1 = Vec3r::Zero();

        Vec3r dCor_dor1;
        if (dor.norm() < CONSTRAINT_EPS)
        {
            dCor_dor1 = Vec3r(1,0,0);
        }
        else
        {
            dCor_dor1 = dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * Math::Skew3(joint_or2.col(2)) * _particles[0]->orientation;
        }


        SingleParticleGradientMatType grad;
        grad.block<1,3>(0,0) = dCp_dp1;
        grad.block<1,3>(0,3) = dCp_dor1;
        grad.block<1,3>(1,0) = dCor_dp1;
        grad.block<1,3>(1,3) = dCor_dor1;

        return grad;
    }
    else if (node_ptr == _particles[1])
    {
        Vec3r dCp_dp2, dCp_dor2;
        if (dp.norm() < CONSTRAINT_EPS)
        {
            dCp_dp2 = Vec3r(1,0,0);
            dCp_dor2 = Vec3r(1,0,0);
        }
        else
        {
            dCp_dp2 = dp/dp.norm();
            dCp_dor2 = -dp.transpose()/dp.norm() * _particles[1]->orientation * Math::Skew3(_r2);
        }
        
        const Vec3r dCor_dp2 = Vec3r::Zero();

        Vec3r dCor_dor2;
        if (dor.norm() < CONSTRAINT_EPS)
        {
            dCor_dor2 = Vec3r(1,0,0);
        }
        else
        {
            dCor_dor2 = -dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or1.transpose() * _particles[1]->orientation * Math::Skew3(_or2.col(2));
        }
        


        SingleParticleGradientMatType grad;
        grad.block<1,3>(0,0) = dCp_dp2;
        grad.block<1,3>(0,3) = dCp_dor2;
        grad.block<1,3>(1,0) = dCor_dp2;
        grad.block<1,3>(1,3) = dCor_dor2;

        return grad;
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

OneSidedRevoluteJointConstraint::OneSidedRevoluteJointConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<5,1>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r1(joint_pos), _or1(joint_or)
{

}

OneSidedRevoluteJointConstraint::ConstraintVecType OneSidedRevoluteJointConstraint::evaluate() const
{
    const Vec3r joint_pos = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r dp = _base_pos - joint_pos;

    const Mat3r joint_or = _particles[0]->orientation * _or1;
    // const Vec3r dtheta = Math::Minus_SO3(_base_or, joint_or);
    const Vec3r joint_axis1 = _particles[0]->orientation * _or1.col(2);
    const Vec3r joint_axis2 = _base_or.col(2);
    const Vec3r a1_cross_a2 = joint_axis1.cross(joint_axis2);
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);

    ConstraintVecType C_vec;
    C_vec.head<3>() = dp;
    C_vec.tail<2>() = dor.head<2>();

    return C_vec;
}

OneSidedRevoluteJointConstraint::GradientMatType OneSidedRevoluteJointConstraint::gradient(bool update_cache) const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Mat3r dCp_dp = -Mat3r::Identity();
    const Mat3r dCp_dor = _particles[0]->orientation * Math::Skew3(_r1);

    // gradients of rotational constraints
    const Mat3r dCor_dp = Mat3r::Zero();

    const Mat3r joint_or = _particles[0]->orientation * _or1;
    const Mat3r dCor_dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * Math::Skew3(_base_or.col(2)) * _particles[0]->orientation;


    grad.block<3,3>(0,0) = dCp_dp;
    grad.block<3,3>(0,3) = dCp_dor;
    grad.block<2,3>(3,0) = dCor_dp.block<2,3>(0,0);
    grad.block<2,3>(3,3) = dCor_dor.block<2,3>(0,0);

    // update cache if specified to
    if (update_cache)
    {
        _cached_gradients[0] = grad.block<5,6>(0,0);
    }
    

    return grad;

}

OneSidedRevoluteJointConstraint::SingleParticleGradientMatType OneSidedRevoluteJointConstraint::singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache) const
{
    if (use_cache)
    {
        if (particle_ptr == _particles[0])
        {
            return _cached_gradients[0];
        }
    }

    if (particle_ptr == _particles[0])
    {
        return gradient(false);
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NormedOneSidedRevoluteJointConstraint::NormedOneSidedRevoluteJointConstraint(
    const Vec3r& base_pos, const Mat3r& base_or,
    SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
)
    : XPBDConstraint<2,1>({particle}, AlphaVecType::Zero()), _base_pos(base_pos), _base_or(base_or), _r1(joint_pos), _or1(joint_or)
{

}

NormedOneSidedRevoluteJointConstraint::ConstraintVecType NormedOneSidedRevoluteJointConstraint::evaluate() const
{
    const Vec3r joint_pos = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r dp = _base_pos - joint_pos;

    const Mat3r joint_or = _particles[0]->orientation * _or1;
    const Vec3r joint_axis1 = _particles[0]->orientation * _or1.col(2);
    const Vec3r joint_axis2 = _base_or.col(2);
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);

    ConstraintVecType C_vec;
    C_vec[0] = dp.norm();
    C_vec[1] = dor.norm();

    return C_vec;
}

NormedOneSidedRevoluteJointConstraint::GradientMatType NormedOneSidedRevoluteJointConstraint::gradient(bool update_cache) const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Vec3r joint_pos = _particles[0]->position + _particles[0]->orientation * _r1;
    const Vec3r dp = _base_pos - joint_pos;

    Vec3r dCp_dp, dCp_dor;
    if (dp.norm() < CONSTRAINT_EPS)
    {
        dCp_dp = Vec3r(1,0,0);
        dCp_dor = Vec3r(1,0,0);
    }
    else
    {
        dCp_dp = -dp / dp.norm();
        dCp_dor =  dp.transpose()/dp.norm() * _particles[0]->orientation * Math::Skew3(_r1);
    }

    // gradients of rotational constraints
    const Mat3r joint_or = _particles[0]->orientation * _or1;
    // const Vec3r dtheta = Math::Minus_SO3(_base_or, joint_or);
    const Vec3r joint_axis1 = _particles[0]->orientation * _or1.col(2);
    const Vec3r joint_axis2 = _base_or.col(2);
    const Vec3r a1_cross_a2 = joint_axis1.cross(joint_axis2);
    const Vec3r dor = Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * _base_or.col(2);
    
    Vec3r dCor_dor;
    if (dor.norm() < CONSTRAINT_EPS)
    {
        dCor_dor = Vec3r(1,0,0);
    }
    else
    {
        dCor_dor = dor.transpose()/dor.norm() * Math::Skew3(Vec3r(0,0,1)) * joint_or.transpose() * Math::Skew3(_base_or.col(2)) * _particles[0]->orientation;
    }

    grad.block<1,3>(0,0) = dCp_dp;
    grad.block<1,3>(0,3) = dCp_dor;
    grad.block<1,3>(1,0) = Vec3r::Zero();
    grad.block<1,3>(1,3) = dCor_dor;

    // update cache if specified to
    if (update_cache)
    {
        _cached_gradients[0] = grad.block<ConstraintDim,6>(0,0);
    }
    

    return grad;

}

NormedOneSidedRevoluteJointConstraint::SingleParticleGradientMatType NormedOneSidedRevoluteJointConstraint::singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache) const
{
    if (use_cache)
    {
        if (particle_ptr == _particles[0])
        {
            return _cached_gradients[0];
        }
    }

    if (particle_ptr == _particles[0])
    {
        return gradient(false);
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}

} // namespace Constraint