#pragma once
#include "constraint/Constraint.hpp"

namespace Constraint
{

class RodElasticConstraint : public XPBDConstraint<SimObject::OrientedParticle::DOF, 2>
{
    public:

    RodElasticConstraint(SimObject::OrientedParticle* node1, SimObject::OrientedParticle* node2, const AlphaVecType& alpha);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;


    private:
    Real _dl;

};

} // namespace Constraint