#include "constraint/RodElasticConstraint.hpp"
#include "common/math.hpp"

namespace Constraint
{

RodElasticConstraint::RodElasticConstraint(const Rod::XPBDRodNode* node1, const Rod::XPBDRodNode* node2, const AlphaVecType& alpha)
    : XPBDConstraint<Rod::XPBDRodNode::NODE_DOF, 2*Rod::XPBDRodNode::NODE_DOF>(alpha),
     _node1(node1), _node2(node2), _dl( (node2->position - node1->position).norm() )
{

}

RodElasticConstraint::ConstraintVecType RodElasticConstraint::evaluate() const
{
    // computing the constraints for the (ci)th segment, which involves nodes (ci-1) and ci
    const Vec3r dp = _node2->position - _node1->position;
    // std::cout << "dp:\n" << dp << std::endl;
    ConstraintVecType C;
    C(Eigen::seqN(0,3)) = (_node1->orientation.transpose() + _node2->orientation.transpose()) * 0.5 * dp / _dl - Vec3r(0,0,1);
    C(Eigen::seqN(3,3)) = Math::Log_SO3(_node1->orientation.transpose() * _node2->orientation) / _dl;

    return C;
}

RodElasticConstraint::GradientMatType RodElasticConstraint::gradient() const
{
    GradientMatType grad;  // 6x12 matrix
    // computing the constraint gradients for the (ci)th segment, which involves nodes (ci-1) and ci
    const Mat3r dCv_dp_iminus1 = -0.5*(_node1->orientation.transpose() + _node2->orientation.transpose()) / _dl;
    const Mat3r dCv_dp_i = -dCv_dp_iminus1;
    
    const Vec3r dp = _node2->position - _node1->position;
    const Mat3r dCv_dor_iminus1 = 0.5*Math::Skew3(_node1->orientation.transpose() * dp / _dl);
    const Mat3r dCv_dor_i = 0.5*Math::Skew3(_node2->orientation.transpose() * dp / _dl);

    const Vec3r dtheta = Math::Log_SO3(_node1->orientation.transpose() * _node2->orientation);
    const Mat3r jac_inv = Math::ExpMap_Jacobian(dtheta).inverse();
    const Mat3r dCu_dor_iminus1 = -jac_inv / _dl;
    const Mat3r dCu_dor_i = jac_inv / _dl;

    // submatrices in correct spot in overall delC matrix
    grad.block<3,3>(0, 0) = dCv_dp_iminus1;
    grad.block<3,3>(0, 3) = dCv_dor_iminus1;
    grad.block<3,3>(0, 6) = dCv_dp_i;
    grad.block<3,3>(0, 9) = dCv_dor_i;
    grad.block<3,3>(3, 0) = Mat3r::Zero();
    grad.block<3,3>(3, 3) = dCu_dor_iminus1;
    grad.block<3,3>(3, 6) = Mat3r::Zero();
    grad.block<3,3>(3, 9) = dCu_dor_i;

    return grad;
}

} // namespace Constraint