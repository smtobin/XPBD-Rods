#pragma once

#include "constraint/Constraint.hpp"
#include "simobject/rod/RodElement.hpp"
#include "simobject/rod/CubicHermiteRodElement.hpp"

namespace Constraint
{

template<class ElementType>
class RodElasticGaussPointConstraint : public XPBDConstraint<6, ElementType::NumNodes>
{
public:
    constexpr static int NumNodes = ElementType::NumNodes;
    using AlphaVecType = typename XPBDConstraint<6, NumNodes>::AlphaVecType;
    using ConstraintVecType = typename XPBDConstraint<6, NumNodes>::ConstraintVecType;
    using GradientMatType = typename XPBDConstraint<6, NumNodes>::GradientMatType;
    using SingleParticleGradientMatType = typename XPBDConstraint<6, NumNodes>::SingleParticleGradientMatType;

    RodElasticGaussPointConstraint(const ElementType* rod_element, Real s_hat, const AlphaVecType& alpha);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;


private:
    /** The rod element */
    const ElementType* _rod_element;

    /** The location (in the reference [0,1] interval) of the Gaussian quadrature point */
    Real _s_hat;

};

} // namespace Constraint