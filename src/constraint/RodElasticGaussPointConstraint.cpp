#include "constraint/RodElasticGaussPointConstraint.hpp"

namespace Constraint
{

template<int Order>
RodElasticGaussPointConstraint<Order>::RodElasticGaussPointConstraint(const SimObject::RodElement<Order>* rod_element, Real s_hat, const AlphaVecType& alpha)
    : XPBDConstraint<6, Order+1>(rod_element->nodes(), alpha), _s_hat(s_hat)
{
}

template<int Order>
typename RodElasticGaussPointConstraint<Order>::ConstraintVecType RodElasticGaussPointConstraint<Order>::evaluate() const
{
    ConstraintVecType v_and_u = _rod_element->strain(_s_hat);
    v_and_u[2] -= 1; // constraint is (v - e3)
    return v_and_u;
}

template<int Order>
typename RodElasticGaussPointConstraint<Order>::GradientMatType RodElasticGaussPointConstraint<Order>::gradient(bool /* update_cache */) const
{
    return _rod_element->strainGradient(_s_hat);
}

template<int Order>
typename RodElasticGaussPointConstraint<Order>::SingleParticleGradientMatType
RodElasticGaussPointConstraint<Order>::singleParticleGradient(const SimObject::OrientedParticle* /* node_ptr */, bool /* use_cache */) const
{
    throw std::runtime_error("singleParticleGradient not implemented for RodElasticGaussPointConstraint");
    return SingleParticleGradientMatType::Zero();
}

template class RodElasticGaussPointConstraint<1>;
template class RodElasticGaussPointConstraint<2>;

} // namespace Constraint