#include "constraint/RodRigidBodyCollisionConstraint.hpp"

namespace Constraint
{

RodRigidBodyCollisionConstraint::RodRigidBodyCollisionConstraint(
    SimObject::OrientedParticle* p1, SimObject::OrientedParticle* p2,
    Real beta, const Vec3r& cp_local_rod,
    SimObject::OrientedParticle* com_rb, const Vec3r& cp_local_rb,
    const Vec3r& n, Real mu_s, Real mu_d
)
    : XPBDConstraint<1, 3>({p1, p2, com_rb}, 1.0e-10*AlphaVecType::Ones()), 
    _beta(beta), _cp_local_rod(cp_local_rod),  _cp_local_rb(cp_local_rb), _n(n), _mu_s(mu_s), _mu_d(mu_d)
{
}

RodRigidBodyCollisionConstraint::ConstraintVecType RodRigidBodyCollisionConstraint::evaluate() const
{
    // contact point on rod
    const Vec3r R_diff = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r R_local = Math::Plus_SO3(_particles[0]->orientation, _beta*R_diff);
    const Vec3r cp_rod = (1-_beta)*_particles[0]->position + _beta*_particles[1]->position + R_local*_cp_local_rod;

    // contact point on rigid body
    const Vec3r cp_rb = _particles[2]->position + _particles[2]->orientation *  _cp_local_rb;

    ConstraintVecType C;
    C[0] = (cp_rb - cp_rod).dot(_n);
    return 0.1*C;
}

RodRigidBodyCollisionConstraint::GradientMatType RodRigidBodyCollisionConstraint::gradient(bool /*update_cache*/) const
{
    // gradients w.r.t. positions of rod
    Vec3r dC_dp1 = -(1-_beta)*_n;
    Vec3r dC_dp2 = -_beta*_n;

    // gradients w.r.t. orientations of rod
    const Vec3r R_diff1 = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r exp_Rdiff1 = Math::Exp_so3(_beta*R_diff1);
    const Mat3r gam1 = Math::ExpMap_RightJacobian(_beta*R_diff1);
    const Mat3r inv_gam1 = Math::ExpMap_InvRightJacobian(R_diff1);
    const Mat3r beta_gam_inv_gam1 = _beta * gam1 * inv_gam1;
    const Mat3r R_local_skew_cp1 = _particles[0]->orientation * exp_Rdiff1 * Math::Skew3(_cp_local_rod);

    const Vec3r dC_dR1 = _n.transpose() * R_local_skew_cp1 * (exp_Rdiff1 - beta_gam_inv_gam1);
    const Vec3r dC_dR2 = _n.transpose() * R_local_skew_cp1 * beta_gam_inv_gam1;

    // gradients w.r.t. rigid body
    Vec3r dC_dprb = _n;
    Vec3r dC_dRrb = -_n.transpose() * _particles[2]->orientation * Math::Skew3( _cp_local_rb);

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = dC_dR1;
    grad.block<1,3>(0,6) = dC_dp2;
    grad.block<1,3>(0,9) = dC_dR2;
    grad.block<1,3>(0,12) = dC_dprb;
    grad.block<1,3>(0,15) = dC_dRrb;

    return grad;
}

RodRigidBodyCollisionConstraint::SingleParticleGradientMatType RodRigidBodyCollisionConstraint::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for RodRigidBodyCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
}

void RodRigidBodyCollisionConstraint::applyFriction(Real lambda_n) const
{
    // contact point on rod
    const Vec3r R_diff1 = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r exp_Rdiff1 = Math::Exp_so3(_beta*R_diff1);
    const Mat3r R_local1 = _particles[0]->orientation * exp_Rdiff1;
    const Vec3r cp_rod = (1-_beta)*_particles[0]->position + _beta*_particles[1]->position + R_local1*_cp_local_rod;

    // previous location of contact point on rod
    const Vec3r R_diff_prev1 = Math::Minus_SO3(_particles[1]->prev_orientation, _particles[0]->prev_orientation);
    const Mat3r R_local_prev1 = Math::Plus_SO3(_particles[0]->prev_orientation, _beta*R_diff_prev1);
    const Vec3r prev_cp_rod = (1-_beta)*_particles[0]->prev_position + _beta*_particles[1]->prev_position + R_local_prev1*_cp_local_rod;

    // contact point on rigid body
    const Vec3r cp_rb = _particles[2]->position + _particles[2]->orientation * _cp_local_rb;

    // previous location of contact point on rigid body 1
    const Vec3r prev_cp_rb = _particles[2]->prev_position + _particles[2]->prev_orientation * _cp_local_rb;

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = (cp_rb - prev_cp_rb) - (cp_rod - prev_cp_rod);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;

    // compute lambda as if we are undoing the entire movement in the tangent direction
    Vec3r tan_dir = dp_tan.normalized();
    Real C = dp_tan.norm();

    // gradients w.r.t. positions of rod
    Vec3r dC_dp1 = -(1-_beta)*tan_dir;
    Vec3r dC_dp2 = -_beta*tan_dir;

    // gradients w.r.t. orientations of rod
    const Mat3r gam1 = Math::ExpMap_RightJacobian(_beta*R_diff1);
    const Mat3r inv_gam1 = Math::ExpMap_InvRightJacobian(R_diff1);
    const Mat3r beta_gam_inv_gam1 = _beta * gam1 * inv_gam1;
    const Mat3r R_local_skew_cp1 = _particles[0]->orientation * exp_Rdiff1 * Math::Skew3(_cp_local_rod);

    const Mat3r dRcp_dR1 = R_local_skew_cp1 * (exp_Rdiff1 - beta_gam_inv_gam1);
    const Mat3r dRcp_dR2 = R_local_skew_cp1 * beta_gam_inv_gam1;

    const Vec3r dC_dR1 = tan_dir.transpose() * dRcp_dR1;
    const Vec3r dC_dR2 = tan_dir.transpose() * dRcp_dR2;

    const Vec3r dC_dp3 = tan_dir;
    const Vec3r dC_dR3 = -tan_dir.transpose() * _particles[0]->orientation * Math::Skew3(_cp_local_rb);

    GradientMatType delC;
    delC.block<1,3>(0,0) = dC_dp1;
    delC.block<1,3>(0,3) = dC_dR1;
    delC.block<1,3>(0,6) = dC_dp2;
    delC.block<1,3>(0,9) = dC_dR2;
    delC.block<1,3>(0,12) = dC_dp3;
    delC.block<1,3>(0,15) = dC_dR3;

    // std::cout << "  dp_tan: " << dp_tan.transpose() << std::endl;
    // std::cout << "  ||dp_tan||: " << dp_tan.norm() << std::endl;
    // std::cout << "  Tangential dir: " << tan_dir.transpose() << std::endl;
    

    Eigen::Vector<Real, StateDim> inertia_inverse;
    for (int i = 0; i < NumParticles; i++)
    {
        inertia_inverse.template block<6,1>(6*i, 0) = 
            Vec6r(1/_particles[i]->mass, 1/_particles[i]->mass, 1/_particles[i]->mass,
                 1/_particles[i]->Ib[0], 1/_particles[i]->Ib[1], 1/_particles[i]->Ib[2]);
    }

    Real LHS = delC * inertia_inverse.asDiagonal() * delC.transpose();
    Real dlam_tan = -C / LHS;

    // std::cout << "  Nominal dlam_tan: " << dlam_tan << std::endl;

    

    // get last relative velocity between contact points, in the tangential direction
    Vec3r v_cp_rod = (1-_beta)*_particles[0]->lin_velocity + _beta*_particles[1]->lin_velocity + dRcp_dR1*_particles[0]->ang_velocity + dRcp_dR2*_particles[1]->ang_velocity;
    Vec3r v_cp_rb = _particles[2]->lin_velocity + _particles[2]->orientation * Math::Skew3(_particles[2]->ang_velocity) * _cp_local_rb;
    Vec3r v_rel = v_cp_rb - v_cp_rod;
    Vec3r v_rel_tan = v_rel - (v_rel.dot(_n))*_n;

    // determine if static friction should be applied (i.e. dp_tan should be reduced to 0)
    // if relative velocity between contact points in the tangential ~~ 0, then the bodies were static relative to one another last step
    if (v_rel_tan.norm() < 1e-4)
    {
        // if nominal tangent lambda >= coeff of static friction * normal lambda, then dynamic friction should be applied
        // so clamp the tangent lambda to correspond to the dynamic force
        if (dlam_tan >= _mu_s*lambda_n)
            dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
        // else
        //     std::cout << "STATIC FRICTION" << std::endl;
    }
    // bodies were moving relative to each other last step, so dynamic friction should be applied
    else
    {
        // clamp the tangent lambda to correspond to the dynamic force
        dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
    }

    // std::cout << "  Clamped dlam_tan: " << dlam_tan << std::endl;

    // update nodes
    for (int i = 0; i < NumParticles; i++)
    {
        SingleParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam_tan;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _particles[i]->positionUpdate(position_update);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OneSidedRodRigidBodyCollisionConstraint::OneSidedRodRigidBodyCollisionConstraint(
    SimObject::OrientedParticle* p1, SimObject::OrientedParticle* p2,
    Real beta, const Vec3r& cp_local_rod,
    const Vec3r& rb_cp,
    const Vec3r& n,
    Real mu_s, Real mu_d
)
    : XPBDConstraint<1, 2>({p1, p2}, 1.0e-10*AlphaVecType::Ones()),
     _beta(beta), _cp_local_rod(cp_local_rod), _cp(rb_cp), _n(n), _mu_s(mu_s), _mu_d(mu_d)
{
}

OneSidedRodRigidBodyCollisionConstraint::ConstraintVecType OneSidedRodRigidBodyCollisionConstraint::evaluate() const
{
    // contact point on rod
    const Vec3r R_diff = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r R_local = Math::Plus_SO3(_particles[0]->orientation, _beta*R_diff);
    const Vec3r cp_rod = (1-_beta)*_particles[0]->position + _beta*_particles[1]->position + R_local*_cp_local_rod;

    ConstraintVecType C;
    C[0] = (_cp - cp_rod).dot(_n);
    return 0.1*C;
}

OneSidedRodRigidBodyCollisionConstraint::GradientMatType OneSidedRodRigidBodyCollisionConstraint::gradient(bool /*update_cache*/) const
{
    // gradients w.r.t. positions of rod
    Vec3r dC_dp1 = -(1-_beta)*_n;
    Vec3r dC_dp2 = -_beta*_n;

    // gradients w.r.t. orientations of rod
    const Vec3r R_diff1 = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r exp_Rdiff1 = Math::Exp_so3(_beta*R_diff1);
    const Mat3r gam1 = Math::ExpMap_RightJacobian(_beta*R_diff1);
    const Mat3r inv_gam1 = Math::ExpMap_InvRightJacobian(R_diff1);
    const Mat3r beta_gam_inv_gam1 = _beta * gam1 * inv_gam1;
    const Mat3r R_local_skew_cp1 = _particles[0]->orientation * exp_Rdiff1 * Math::Skew3(_cp_local_rod);

    const Vec3r dC_dR1 = _n.transpose() * R_local_skew_cp1 * (exp_Rdiff1 - beta_gam_inv_gam1);
    const Vec3r dC_dR2 = _n.transpose() * R_local_skew_cp1 * beta_gam_inv_gam1;

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = dC_dR1;
    grad.block<1,3>(0,6) = dC_dp2;
    grad.block<1,3>(0,9) = dC_dR2;

    return grad;
}

OneSidedRodRigidBodyCollisionConstraint::SingleParticleGradientMatType OneSidedRodRigidBodyCollisionConstraint::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for OneSidedRodRigidBodyCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
}

void OneSidedRodRigidBodyCollisionConstraint::applyFriction(Real lambda_n) const
{
    // contact point on rod 1
    const Vec3r R_diff1 = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r exp_Rdiff1 = Math::Exp_so3(_beta*R_diff1);
    const Mat3r R_local1 = _particles[0]->orientation * exp_Rdiff1;
    const Vec3r cp_rod = (1-_beta)*_particles[0]->position + _beta*_particles[1]->position + R_local1*_cp_local_rod;

    // previous location of contact point on rod
    const Vec3r R_diff_prev1 = Math::Minus_SO3(_particles[1]->prev_orientation, _particles[0]->prev_orientation);
    const Mat3r R_local_prev1 = Math::Plus_SO3(_particles[0]->prev_orientation, _beta*R_diff_prev1);
    const Vec3r prev_cp_rod = (1-_beta)*_particles[0]->prev_position + _beta*_particles[1]->prev_position + R_local_prev1*_cp_local_rod;

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = -(cp_rod - prev_cp_rod);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;


    // gradients w.r.t. positions of rod 1
    Vec3r tan_dir = dp_tan.normalized();
     Real C = dp_tan.norm();


    Vec3r dC_dp1 = -(1-_beta)*tan_dir;
    Vec3r dC_dp2 = -_beta*tan_dir;

    // gradients w.r.t. orientations of rod 1
    const Mat3r gam1 = Math::ExpMap_RightJacobian(_beta*R_diff1);
    const Mat3r inv_gam1 = Math::ExpMap_InvRightJacobian(R_diff1);
    const Mat3r beta_gam_inv_gam1 = _beta * gam1 * inv_gam1;
    const Mat3r R_local_skew_cp1 = _particles[0]->orientation * exp_Rdiff1 * Math::Skew3(_cp_local_rod);

    const Mat3r dRcp_dR1 = R_local_skew_cp1 * (exp_Rdiff1 - beta_gam_inv_gam1);
    const Mat3r dRcp_dR2 = R_local_skew_cp1 * beta_gam_inv_gam1;

    const Vec3r dC_dR1 = tan_dir.transpose() * dRcp_dR1;
    const Vec3r dC_dR2 = tan_dir.transpose() * dRcp_dR2;

    // compute lambda as if we are undoing the entire movement in the tangent direction
    GradientMatType delC;
    delC.block<1,3>(0,0) = dC_dp1;
    delC.block<1,3>(0,3) = dC_dR1;
    delC.block<1,3>(0,6) = dC_dp2;
    delC.block<1,3>(0,9) = dC_dR2;

    // std::cout << "  dp_tan: " << dp_tan.transpose() << std::endl;
    // std::cout << "  ||dp_tan||: " << dp_tan.norm() << std::endl;
    // std::cout << "  Tangential dir: " << tan_dir.transpose() << std::endl;
    

    Eigen::Vector<Real, StateDim> inertia_inverse;
    for (int i = 0; i < NumParticles; i++)
    {
        inertia_inverse.template block<6,1>(6*i, 0) = 
            Vec6r(1/_particles[i]->mass, 1/_particles[i]->mass, 1/_particles[i]->mass,
                 1/_particles[i]->Ib[0], 1/_particles[i]->Ib[1], 1/_particles[i]->Ib[2]);
    }

    Real LHS = delC * inertia_inverse.asDiagonal() * delC.transpose();
    Real dlam_tan = -C / LHS;

    // std::cout << "  Nominal dlam_tan: " << dlam_tan << std::endl;

    

    // get last relative velocity between contact points, in the tangential direction
    Vec3r v_cp_rod = (1-_beta)*_particles[0]->lin_velocity + _beta*_particles[1]->lin_velocity + dRcp_dR1*_particles[0]->ang_velocity + dRcp_dR2*_particles[1]->ang_velocity;
    Vec3r v_rel = v_cp_rod;
    Vec3r v_rel_tan = v_rel - (v_rel.dot(_n))*_n;

    // determine if static friction should be applied (i.e. dp_tan should be reduced to 0)
    // if relative velocity between contact points in the tangential ~~ 0, then the bodies were static relative to one another last step
    if (v_rel_tan.norm() < 1e-4)
    {
        // if nominal tangent lambda >= coeff of static friction * normal lambda, then dynamic friction should be applied
        // so clamp the tangent lambda to correspond to the dynamic force
        if (dlam_tan >= _mu_s*lambda_n)
            dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
        // else
            // std::cout << "STATIC FRICTION" << std::endl;
    }
    // bodies were moving relative to each other last step, so dynamic friction should be applied
    else
    {
        // clamp the tangent lambda to correspond to the dynamic force
        dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
    }

    // std::cout << "  Clamped dlam_tan: " << dlam_tan << std::endl;

    // update nodes
    for (int i = 0; i < NumParticles; i++)
    {
        SingleParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam_tan;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _particles[i]->positionUpdate(position_update);
    }
}


} // namespace Constraint