#ifndef __ROD_ELASTIC_CONSTRAINT_HPP
#define __ROD_ELASTIC_CONSTRAINT_HPP

#include "constraint/Constraint.hpp"

namespace Constraint
{

class RodElasticConstraint : public XPBDConstraint<SimObject::OrientedParticle::DOF, 2>
{
    public:

    RodElasticConstraint(int node_index1, const SimObject::OrientedParticle* node1, int node_index2, const SimObject::OrientedParticle* node2, const AlphaVecType& alpha);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleNodeGradient(int node_index, bool use_cache=false) const override;


    private:
    const SimObject::OrientedParticle* _node1;
    const SimObject::OrientedParticle* _node2;
    const Real _dl;

};

} // namespace Constraint

#endif // __ROD_ELASTIC_CONSTRAINT_HPP