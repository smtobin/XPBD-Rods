#pragma once

#include "constraint/Constraint.hpp"
#include "simobject/rod/RodElement.hpp"

namespace Constraint
{

template<int Order>
class RodElasticGaussPointConstraint : public XPBDConstraint<6, SimObject::RodElement<Order>::NumNodes>
{
public:
    constexpr static int NumNodes = SimObject::RodElement<Order>::NumNodes;
    using AlphaVecType = typename XPBDConstraint<6, NumNodes>::AlphaVecType;
    using ConstraintVecType = typename XPBDConstraint<6, NumNodes>::ConstraintVecType;
    using GradientMatType = typename XPBDConstraint<6, NumNodes>::GradientMatType;
    using SingleParticleGradientMatType = typename XPBDConstraint<6, NumNodes>::SingleParticleGradientMatType;

    RodElasticGaussPointConstraint(const SimObject::RodElement<Order>* rod_element, Real s_hat, const AlphaVecType& alpha);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;


private:
    /** The rod element */
    const SimObject::RodElement<Order>* _rod_element;

    /** The location (in the reference [0,1] interval) of the Gaussian quadrature point */
    Real _s_hat;

};

} // namespace Constraint