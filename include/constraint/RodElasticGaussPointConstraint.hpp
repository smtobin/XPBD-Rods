#pragma once

#include "constraint/Constraint.hpp"
#include "simobject/rod/RodElement.hpp"
#include "simobject/rod/CubicHermiteRodElement.hpp"

namespace Constraint
{

template<class ElementType>
class RodElasticGaussPointConstraint : public XPBDConstraint<6, ElementType::NumNodes, 2*ElementType::NumNodeDerivatives>
{
public:
    using BaseConstraintType = XPBDConstraint<6, ElementType::NumNodes, 2*ElementType::NumNodeDerivatives>;
    
    constexpr static int NumNodes = ElementType::NumNodes;
    using AlphaVecType = typename BaseConstraintType::AlphaVecType;
    using ConstraintVecType = typename BaseConstraintType::ConstraintVecType;
    using GradientMatType = typename BaseConstraintType::GradientMatType;

    RodElasticGaussPointConstraint(const ElementType* rod_element, Real s_hat, const AlphaVecType& alpha);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    /** The rod element */
    const ElementType* _rod_element;

    /** The location (in the reference [0,1] interval) of the Gaussian quadrature point */
    Real _s_hat;

};

} // namespace Constraint