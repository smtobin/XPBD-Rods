#include "constraint/RodRigidBodyCollisionConstraint.hpp"

namespace Constraint
{

RodRigidBodyCollisionConstraint::RodRigidBodyCollisionConstraint(
    SimObject::OrientedParticle* p1, SimObject::OrientedParticle* p2,
    Real beta, Real r_rod,
    SimObject::OrientedParticle* com_rb, const Vec3r& r_rb,
    const Vec3r& n
)
    : XPBDConstraint<1, 3>({p1, p2, com_rb}, 1.0e-10*AlphaVecType::Ones()), _beta(beta), _r_rod(r_rod), _r_rb(r_rb), _n(n)
{
}

RodRigidBodyCollisionConstraint::ConstraintVecType RodRigidBodyCollisionConstraint::evaluate() const
{
    // contact point on rod
    const Vec3r cp_rod = (1-_beta)*_particles[0]->position + _beta*_particles[1]->position + _n*_r_rod;
    // contact point on rigid body
    const Vec3r cp_rb = _particles[2]->position + _particles[2]->orientation * _r_rb;

    ConstraintVecType C;
    C[0] = (cp_rb - cp_rod).dot(_n);
    return C;
}

RodRigidBodyCollisionConstraint::GradientMatType RodRigidBodyCollisionConstraint::gradient(bool /*update_cache*/) const
{
    // gradients w.r.t. positions of rod
    Vec3r dC_dp1 = -(1-_beta)*_n;
    Vec3r dC_dp2 = -_beta*_n;

    // gradients w.r.t. rigid body
    Vec3r dC_dprb = _n;
    Vec3r dC_dRrb = -_n.transpose() * _particles[2]->orientation * Math::Skew3(_r_rb);

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = Vec3r::Zero();
    grad.block<1,3>(0,6) = dC_dp2;
    grad.block<1,3>(0,9) = Vec3r::Zero();
    grad.block<1,3>(0,12) = dC_dprb;
    grad.block<1,3>(0,15) = dC_dRrb;

    return grad;
}

RodRigidBodyCollisionConstraint::SingleParticleGradientMatType RodRigidBodyCollisionConstraint::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for RodRigidBodyCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
}

} // namespace Constraint