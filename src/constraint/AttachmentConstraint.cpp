#include "constraint/AttachmentConstraint.hpp"
#include "common/math.hpp"

namespace Constraint
{

AttachmentConstraint::AttachmentConstraint(int node_index, const SimObject::OrientedParticle* node, const AlphaVecType& alpha, const Vec3r& position, const Mat3r& orientation)
    : XPBDConstraint<SimObject::OrientedParticle::DOF, 1>(alpha),
    _node(node), _ref_position(position), _ref_orientation(orientation)
{
    _node_indices[0] = node_index;
}

AttachmentConstraint::ConstraintVecType AttachmentConstraint::evaluate() const 
{
    // fixed base constraint
    ConstraintVecType C;
    C( Eigen::seqN(0,3) ) = _node->position - _ref_position;
    C( Eigen::seqN(3,3) ) = Math::Minus_SO3(_node->orientation, _ref_orientation);
    return C;
}

AttachmentConstraint::GradientMatType AttachmentConstraint::gradient(bool update_cache) const
{
    GradientMatType grad = GradientMatType::Zero();
    // fixed base constraint gradient
    const Vec3r dtheta0 = Math::Minus_SO3(_node->orientation, _ref_orientation);
    const Mat3r jac_inv0 = Math::ExpMap_Jacobian(dtheta0).inverse();
    grad.block<3,3>(0,0) = Mat3r::Identity();
    grad.block<3,3>(3,3) = jac_inv0;

    if (update_cache)
    {
        _cached_gradients[0] = grad;
    }

    return grad;
}

AttachmentConstraint::SingleParticleGradientMatType AttachmentConstraint::singleNodeGradient(int node_index, bool use_cache) const
{
    if (node_index == _node_indices[0])
    {
        if (use_cache)
            return _cached_gradients[0];
        
        SingleParticleGradientMatType grad = SingleParticleGradientMatType::Zero();
        // fixed base constraint gradient
        const Vec3r dtheta0 = Math::Minus_SO3(_node->orientation, _ref_orientation);
        const Mat3r jac_inv0 = Math::ExpMap_Jacobian(dtheta0).inverse();
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