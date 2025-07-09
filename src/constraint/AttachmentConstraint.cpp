#include "constraint/AttachmentConstraint.hpp"
#include "common/math.hpp"

namespace Constraint
{

AttachmentConstraint::AttachmentConstraint(const Rod::XPBDRodNode* node, const AlphaVecType& alpha, const Vec3r& position, const Mat3r& orientation)
    : XPBDConstraint<Rod::XPBDRodNode::NODE_DOF, 1>(alpha),
    _node(node), _ref_position(position), _ref_orientation(orientation)
{
    _node_indices[0] = _node->index;
}

AttachmentConstraint::ConstraintVecType AttachmentConstraint::evaluate() const 
{
    // fixed base constraint
    ConstraintVecType C;
    C( Eigen::seqN(0,3) ) = _node->position - _ref_position;
    C( Eigen::seqN(3,3) ) = Math::Minus_SO3(_node->orientation, _ref_orientation);
    return C;
}

AttachmentConstraint::GradientMatType AttachmentConstraint::gradient() const
{
    GradientMatType grad = GradientMatType::Zero();
    // fixed base constraint gradient
    const Vec3r dtheta0 = Math::Minus_SO3(_node->orientation, _ref_orientation);
    const Mat3r jac_inv0 = Math::ExpMap_Jacobian(dtheta0).inverse();
    grad.block<3,3>(0,0) = Mat3r::Identity();
    grad.block<3,3>(3,3) = jac_inv0;

    return grad;
}

AttachmentConstraint::SingleNodeGradientMatType AttachmentConstraint::singleNodeGradient(int node_index) const
{
    if (node_index == _node->index)
    {
        SingleNodeGradientMatType grad = SingleNodeGradientMatType::Zero();
        // fixed base constraint gradient
        const Vec3r dtheta0 = Math::Minus_SO3(_node->orientation, _ref_orientation);
        const Mat3r jac_inv0 = Math::ExpMap_Jacobian(dtheta0).inverse();
        grad.block<3,3>(0,0) = Mat3r::Identity();
        grad.block<3,3>(3,3) = jac_inv0;

        return grad;
    }
    else
    {
        return SingleNodeGradientMatType::Zero();
    }
}

} // namespace Constraint