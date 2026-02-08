#include "constraint/RigidBodyCollisionConstraint.hpp"

namespace Constraint
{

RigidBodyCollisionConstraint::RigidBodyCollisionConstraint(
    SimObject::OrientedParticle* com1, const Vec3r& r1,
    SimObject::OrientedParticle* com2, const Vec3r& r2,
    const Vec3r& n
)
    : XPBDConstraint<1, 2>({com1, com2}, 1.0e-10*AlphaVecType::Ones()), _r1(r1), _r2(r2), _n(n)
{
}

RigidBodyCollisionConstraint::ConstraintVecType RigidBodyCollisionConstraint::evaluate() const
{
    // contact point on rigid body 1
    const Vec3r cp1 = _particles[0]->position + _particles[0]->orientation * _r1;
    // contact point on rigid body 2
    const Vec3r cp2 = _particles[1]->position + _particles[1]->orientation * _r2;

    ConstraintVecType C;
    C[0] = (cp2 - cp1).dot(_n);
    return C;
}

RigidBodyCollisionConstraint::GradientMatType RigidBodyCollisionConstraint::gradient(bool /*update_cache*/) const
{
    // positional gradients
    Vec3r dC_dp1 = -_n;
    Vec3r dC_dp2 = _n;

    // orientation gradients
    Vec3r dC_dR1 = _n.transpose() * _particles[0]->orientation * Math::Skew3(_r1);
    Vec3r dC_dR2 = -_n.transpose() * _particles[1]->orientation * Math::Skew3(_r2);

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = dC_dR1;
    grad.block<1,3>(0,6) = dC_dp2;
    grad.block<1,3>(0,9) = dC_dR2;

    return grad;
}

RigidBodyCollisionConstraint::SingleParticleGradientMatType RigidBodyCollisionConstraint::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for RigidBodyCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


OneSidedRigidBodyCollisionConstraint::OneSidedRigidBodyCollisionConstraint(
    const Vec3r& cp,
    SimObject::OrientedParticle* com1, const Vec3r& r1,
    const Vec3r& n
)
    : XPBDConstraint<1, 1>({com1}, 1.0e-10*AlphaVecType::Ones()), _r1(r1), _cp(cp), _n(n)
{
}

OneSidedRigidBodyCollisionConstraint::ConstraintVecType OneSidedRigidBodyCollisionConstraint::evaluate() const
{
    // contact point on rigid body 1
    const Vec3r cp1 = _particles[0]->position + _particles[0]->orientation * _r1;

    ConstraintVecType C;
    C[0] = (cp1 - _cp).dot(_n);
    return C;
}

OneSidedRigidBodyCollisionConstraint::GradientMatType OneSidedRigidBodyCollisionConstraint::gradient(bool /*update_cache*/) const
{
    // positional gradients
    Vec3r dC_dp1 = _n;

    // orientation gradients
    Vec3r dC_dR1 = -_n.transpose() * _particles[0]->orientation * Math::Skew3(_r1);

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = dC_dR1;

    return grad;
}

OneSidedRigidBodyCollisionConstraint::SingleParticleGradientMatType OneSidedRigidBodyCollisionConstraint::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for OneSidedRigidBodyCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
}

} // namespace Collision