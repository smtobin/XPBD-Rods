#include "constraint/RodRodCollisionConstraint.hpp"

namespace Constraint
{

RodRodCollisionConstraint::RodRodCollisionConstraint(
    SimObject::OrientedParticle* p1_rod1, SimObject::OrientedParticle* p2_rod1,
    Real beta1, Real r_rod1,
    SimObject::OrientedParticle* p1_rod2, SimObject::OrientedParticle* p2_rod2,
    Real beta2, Real r_rod2,
    const Vec3r& n
)
    : XPBDConstraint<1, 4>({p1_rod1, p2_rod1, p1_rod2, p2_rod2}, 1.0e-10*AlphaVecType::Ones()),
     _beta1(beta1), _beta2(beta2), _r_rod1(r_rod1), _r_rod2(r_rod2), _n(n)
{
}

RodRodCollisionConstraint::ConstraintVecType RodRodCollisionConstraint::evaluate() const
{
    // contact point on rod 1
    const Vec3r cp_rod1 = (1-_beta1)*_particles[0]->position + _beta1*_particles[1]->position + _n*_r_rod1;
    // contact point on rod 2
    const Vec3r cp_rod2 = (1-_beta2)*_particles[2]->position + _beta2*_particles[3]->position - _n*_r_rod2;

    ConstraintVecType C;
    C[0] = (cp_rod2 - cp_rod1).dot(_n);
    return C;
}

RodRodCollisionConstraint::GradientMatType RodRodCollisionConstraint::gradient(bool /*update_cache*/) const
{
    // gradients w.r.t. positions of rod 1
    Vec3r dC_dp1 = -(1-_beta1)*_n;
    Vec3r dC_dp2 = -_beta1*_n;

    // gradients w.r.t. positions of rod 2
    Vec3r dC_dp3 = (1-_beta2)*_n;
    Vec3r dC_dp4 = (_beta2)*_n;

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = Vec3r::Zero();
    grad.block<1,3>(0,6) = dC_dp2;
    grad.block<1,3>(0,9) = Vec3r::Zero();
    grad.block<1,3>(0,12) = dC_dp3;
    grad.block<1,3>(0,15) = Vec3r::Zero();
    grad.block<1,3>(0,18) = dC_dp4;
    grad.block<1,3>(0,21) = Vec3r::Zero();

    return grad;
}

RodRodCollisionConstraint::SingleParticleGradientMatType RodRodCollisionConstraint::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for RodRodCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
}

} // namespace Constraint