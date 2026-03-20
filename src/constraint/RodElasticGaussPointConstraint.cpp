#include "constraint/RodElasticGaussPointConstraint.hpp"

namespace Constraint
{

template<class ElementType>
RodElasticGaussPointConstraint<ElementType>::RodElasticGaussPointConstraint(const ElementType* rod_element, Real s_hat, const AlphaVecType& alpha)
    : XPBDConstraint<6, NumNodes, 0>(rod_element->nodes(), alpha), _rod_element(rod_element), _s_hat(s_hat)
{
}

template<class ElementType>
typename RodElasticGaussPointConstraint<ElementType>::ConstraintVecType RodElasticGaussPointConstraint<ElementType>::evaluate() const
{
    return _rod_element->strain(_s_hat);
}

template<class ElementType>
typename RodElasticGaussPointConstraint<ElementType>::GradientMatType RodElasticGaussPointConstraint<ElementType>::gradient() const
{
    return _rod_element->strainGradient(_s_hat);
}

template class RodElasticGaussPointConstraint<SimObject::RodElement<0>>;
template class RodElasticGaussPointConstraint<SimObject::RodElement<1>>;
template class RodElasticGaussPointConstraint<SimObject::RodElement<2>>;
template class RodElasticGaussPointConstraint<SimObject::CubicHermiteRodElement>;

} // namespace Constraint