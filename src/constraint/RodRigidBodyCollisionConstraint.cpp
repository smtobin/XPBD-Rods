#include "constraint/RodRigidBodyCollisionConstraint.hpp"

namespace Constraint
{

RodRigidBodyCollisionConstraint::RodRigidBodyCollisionConstraint(
    SimObject::OrientedParticle* p1, SimObject::OrientedParticle* p2,
    Real beta, const Vec3r& cp_local_rod,
    SimObject::OrientedParticle* com_rb, const Vec3r& cp_local_rb,
    const Vec3r& n
)
    : XPBDConstraint<1, 3>({p1, p2, com_rb}, 1.0e-10*AlphaVecType::Ones()), _beta(beta), _cp_local_rod(cp_local_rod),  _cp_local_rb(cp_local_rb), _n(n)
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

void RodRigidBodyCollisionConstraint::applyFriction(Real lambda_n, Real mu_s, Real mu_d) const
{
    // // contact point on rod
    // const Vec3r cp_rod = (1-_beta)*_particles[0]->position + _beta*_particles[1]->position + _n*_r_rod;
    // // contact point on rigid body
    // const Vec3r cp_rb = _particles[2]->position + _particles[2]->orientation *  _cp_local_rb;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OneSidedRodRigidBodyCollisionConstraint::OneSidedRodRigidBodyCollisionConstraint(
    SimObject::OrientedParticle* p1, SimObject::OrientedParticle* p2,
    Real beta, const Vec3r& cp_local_rod,
    const Vec3r& rb_cp,
    const Vec3r& n
)
    : XPBDConstraint<1, 2>({p1, p2}, 1.0e-10*AlphaVecType::Ones()), _beta(beta), _cp_local_rod(cp_local_rod), _cp(rb_cp), _n(n)
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


} // namespace Constraint