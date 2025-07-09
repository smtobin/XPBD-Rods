#ifndef __ROD_ELASTIC_CONSTRAINT_HPP
#define __ROD_ELASTIC_CONSTRAINT_HPP

#include "constraint/Constraint.hpp"
#include "rod/XPBDRodNode.hpp"

namespace Constraint
{

class RodElasticConstraint : public XPBDConstraint<Rod::XPBDRodNode::NODE_DOF, 2>
{
    public:

    RodElasticConstraint(const Rod::XPBDRodNode* node1, const Rod::XPBDRodNode* node2, const AlphaVecType& alpha);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleNodeGradientMatType singleNodeGradient(int node_index, bool use_cache=false) const override;


    private:
    const Rod::XPBDRodNode* _node1;
    const Rod::XPBDRodNode* _node2;
    const Real _dl;

};

} // namespace Constraint

#endif // __ROD_ELASTIC_CONSTRAINT_HPP