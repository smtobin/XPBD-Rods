#include "constraint/RodMidElementFixedConstraint.hpp"

namespace Constraint
{

template<class ElementType>
RodMidElementFixedConstraint<ElementType>::RodMidElementFixedConstraint(const ElementType* rod_element, Real s_hat, const Vec3r& p_ref, const Mat3r& R_ref, const AlphaVecType& alpha, Real beta)
    : XPBDConstraint<6, ElementType::NumNodes, 0>(
        rod_element->nodes(),
        alpha, beta),
     _rod_element(rod_element),
      _s_hat(s_hat),
      _p_ref(p_ref),
      _R_ref(R_ref)
{
}

template<class ElementType>
typename RodMidElementFixedConstraint<ElementType>::ConstraintVecType RodMidElementFixedConstraint<ElementType>::evaluate() const
{
    Vec6r C;
    C.template block<3,1>(0,0) = _rod_element->position(_s_hat) - _p_ref;
    C.template block<3,1>(3,0) = Math::Minus_SO3( _rod_element->orientation(_s_hat), _R_ref);
    return C;
}

template<class ElementType>
typename RodMidElementFixedConstraint<ElementType>::GradientMatType RodMidElementFixedConstraint<ElementType>::gradient() const
{
    GradientMatType grad = GradientMatType::Zero();

    // positional gradients
    for (int i = 0; i < ElementType::NumNodes; i++)
    {
        grad.template block<3,3>(0, 6*i) = _rod_element->Ni(i, _s_hat) * Mat3r::Identity();
    }

    // rotational gradients
    typename ElementType::OrientationGradientMatType dR_dRi = _rod_element->orientationGradient(_s_hat);
    Vec3r theta = Math::Minus_SO3( _rod_element->orientation(_s_hat), _R_ref);
    Mat3r gam_inv = Math::ExpMap_InvRightJacobian(theta);
    for (int i = 0; i < ElementType::NumNodes; i++)
    {
        grad.template block<3,3>(3, 6*i+3) = gam_inv * dR_dRi.template block<3,3>(0,3*i);
    }

    return grad;
}

template class RodMidElementFixedConstraint<SimObject::RodElement<0>>;
template class RodMidElementFixedConstraint<SimObject::RodElement<1>>;
template class RodMidElementFixedConstraint<SimObject::RodElement<2>>;
template class RodMidElementFixedConstraint<SimObject::RodElement<3>>;
// template class RodMidElementFixedConstraint<SimObject::CubicHermiteRodElement>;

} // namespace Constraint