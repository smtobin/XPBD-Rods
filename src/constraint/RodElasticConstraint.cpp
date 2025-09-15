#include "constraint/RodElasticConstraint.hpp"
#include "common/math.hpp"

namespace Constraint
{

RodElasticConstraint::RodElasticConstraint(int node_index1, const SimObject::OrientedParticle* node1, int node_index2, const SimObject::OrientedParticle* node2, const AlphaVecType& alpha)
    : XPBDConstraint<SimObject::OrientedParticle::DOF, 2>(alpha),
     _node1(node1), _node2(node2), _dl( (node2->position - node1->position).norm() )
{
    _node_indices[0] = node_index1;
    _node_indices[1] = node_index2;
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

RodElasticConstraint::GradientMatType RodElasticConstraint::gradient(bool update_cache) const
{
    GradientMatType grad;  // 6x12 matrix
    // computing the constraint gradients for the (ci)th segment, which involves nodes (ci-1) and ci
    const Mat3r dCv_dp1 = -0.5*(_node1->orientation.transpose() + _node2->orientation.transpose()) / _dl;
    const Mat3r dCv_dp2 = -dCv_dp1;
    
    const Vec3r dp = _node2->position - _node1->position;
    const Mat3r dCv_dor1 = 0.5*Math::Skew3(_node1->orientation.transpose() * dp / _dl);
    const Mat3r dCv_dor2 = 0.5*Math::Skew3(_node2->orientation.transpose() * dp / _dl);

    const Vec3r dtheta = Math::Log_SO3(_node1->orientation.transpose() * _node2->orientation);
    const Mat3r jac_inv = Math::ExpMap_Jacobian(dtheta).inverse();
    const Mat3r dCu_dor1 = -jac_inv / _dl;
    const Mat3r dCu_dor2 = jac_inv / _dl;

    // submatrices in correct spot in overall delC matrix
    grad.block<3,3>(0, 0) = dCv_dp1;
    grad.block<3,3>(0, 3) = dCv_dor1;
    grad.block<3,3>(0, 6) = dCv_dp2;
    grad.block<3,3>(0, 9) = dCv_dor2;
    grad.block<3,3>(3, 0) = Mat3r::Zero();
    grad.block<3,3>(3, 3) = dCu_dor1;
    grad.block<3,3>(3, 6) = Mat3r::Zero();
    grad.block<3,3>(3, 9) = dCu_dor2;

    // update cache if specified to
    if (update_cache)
    {
        _cached_gradients[0] = grad.block<6,6>(0,0);
        _cached_gradients[1] = grad.block<6,6>(0,6);
    }
    

    return grad;
}

RodElasticConstraint::SingleParticleGradientMatType RodElasticConstraint::singleNodeGradient(int node_index, bool use_cache) const
{
    if (use_cache)
    {
        if (node_index == _node_indices[0])
        {
            return _cached_gradients[0];
        }
        else if (node_index == _node_indices[1])
        {
            return _cached_gradients[1];
        }
    }

    if (node_index == _node_indices[0])
    {
        const Mat3r dCv_dp1 = -0.5*(_node1->orientation.transpose() + _node2->orientation.transpose()) / _dl;
        const Vec3r dp = _node2->position - _node1->position;
        const Mat3r dCv_dor1 = 0.5*Math::Skew3(_node1->orientation.transpose() * dp / _dl);

        const Vec3r dtheta = Math::Log_SO3(_node1->orientation.transpose() * _node2->orientation);
        const Mat3r jac_inv = Math::ExpMap_Jacobian(dtheta).inverse();
        const Mat3r dCu_dor1 = -jac_inv / _dl;

        SingleParticleGradientMatType grad;
        grad.block<3,3>(0,0) = dCv_dp1;
        grad.block<3,3>(0,3) = dCv_dor1;
        grad.block<3,3>(3,0) = Mat3r::Zero();
        grad.block<3,3>(3,3) = dCu_dor1;
        return grad;
    }
    else if (node_index == _node_indices[1])
    {
        const Mat3r dCv_dp2 = 0.5*(_node1->orientation.transpose() + _node2->orientation.transpose()) / _dl;
        const Vec3r dp = _node2->position - _node1->position;
        const Mat3r dCv_dor2 = 0.5*Math::Skew3(_node2->orientation.transpose() * dp / _dl);
        const Vec3r dtheta = Math::Log_SO3(_node1->orientation.transpose() * _node2->orientation);
        const Mat3r jac_inv = Math::ExpMap_Jacobian(dtheta).inverse();
        const Mat3r dCu_dor2 = jac_inv / _dl;

        SingleParticleGradientMatType grad;
        grad.block<3,3>(0,0) = dCv_dp2;
        grad.block<3,3>(0,3) = dCv_dor2;
        grad.block<3,3>(3,0) = Mat3r::Zero();
        grad.block<3,3>(3,3) = dCu_dor2;
        return grad;
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}

} // namespace Constraint