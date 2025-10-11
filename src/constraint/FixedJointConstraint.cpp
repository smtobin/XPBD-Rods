#include "constraint/FixedJointConstraint.hpp"
#include "common/math.hpp"

namespace Constraint
{

FixedJointConstraint::FixedJointConstraint(SimObject::OrientedParticle* node, const AlphaVecType& alpha, const Vec3r& position, const Mat3r& orientation)
    : XPBDConstraint<SimObject::OrientedParticle::DOF, 1>({node}, alpha),
    _ref_position(position), _ref_orientation(orientation)
{
}

FixedJointConstraint::ConstraintVecType FixedJointConstraint::evaluate() const 
{
    // fixed base constraint
    ConstraintVecType C;
    C( Eigen::seqN(0,3) ) = _particles[0]->position - _ref_position;
    C( Eigen::seqN(3,3) ) = Math::Minus_SO3(_particles[0]->orientation, _ref_orientation);
    return C;
}

FixedJointConstraint::GradientMatType FixedJointConstraint::gradient(bool update_cache) const
{
    GradientMatType grad = GradientMatType::Zero();
    // fixed base constraint gradient
    const Vec3r dtheta0 = Math::Minus_SO3(_particles[0]->orientation, _ref_orientation);
    const Mat3r jac_inv0 = Math::ExpMap_InvRightJacobian(dtheta0);
    grad.block<3,3>(0,0) = Mat3r::Identity();
    grad.block<3,3>(3,3) = jac_inv0;

    if (update_cache)
    {
        _cached_gradients[0] = grad;
    }

    return grad;
}

FixedJointConstraint::SingleParticleGradientMatType FixedJointConstraint::singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache) const
{
    if (particle_ptr == _particles[0])
    {
        if (use_cache)
            return _cached_gradients[0];
        
        SingleParticleGradientMatType grad = SingleParticleGradientMatType::Zero();
        // fixed base constraint gradient
        const Vec3r dtheta0 = Math::Minus_SO3(_particles[0]->orientation, _ref_orientation);
        const Mat3r jac_inv0 = Math::ExpMap_InvRightJacobian(dtheta0);
        grad.block<3,3>(0,0) = Mat3r::Identity();
        grad.block<3,3>(3,3) = jac_inv0;

        return grad;
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}

} // namespace Constraint