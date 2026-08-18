#pragma once

#include "constraint/Constraint.hpp"
#include "simobject/rod/RodElement.hpp"

namespace Constraint
{

template<class ElementType>
class RodMidElementFixedConstraint : public XPBDConstraint<6, ElementType::NumNodes, 0>
{
public:
    using BaseConstraintType = XPBDConstraint<6, ElementType::NumNodes, 0>;
    
    constexpr static int NumNodes = ElementType::NumNodes;
    using AlphaVecType = typename BaseConstraintType::AlphaVecType;
    using ConstraintVecType = typename BaseConstraintType::ConstraintVecType;
    using GradientMatType = typename BaseConstraintType::GradientMatType;

    RodMidElementFixedConstraint(const ElementType* rod_element, Real s_hat, const Vec3r& p_ref, const Mat3r& R_ref, const AlphaVecType& alpha, Real beta=0);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    /** The rod element */
    const ElementType* _rod_element;

    /** The location (in the reference [0,1] interval) of the Gaussian quadrature point */
    Real _s_hat;

    /** The reference position and orientation */
    Vec3r _p_ref;
    Mat3r _R_ref;

};

} // namespace Constraint