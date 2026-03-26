#include "constraint/RodElasticGaussPointConstraint.hpp"
#include "common/ArrayConcatenate.hpp"

namespace Constraint
{

template<class ElementType>
RodElasticGaussPointConstraint<ElementType>::RodElasticGaussPointConstraint(const ElementType* rod_element, Real s_hat, const AlphaVecType& alpha)
    : XPBDConstraint<6, ElementType::NumNodes, 2*ElementType::NumNodeDerivatives>(
        rod_element->nodes(),
        rod_element->nodeDerivatives(),
        alpha),
     _rod_element(rod_element), _s_hat(s_hat)
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
template class RodElasticGaussPointConstraint<SimObject::RodElement<3>>;
template class RodElasticGaussPointConstraint<SimObject::CubicHermiteRodElement>;

} // namespace Constraint