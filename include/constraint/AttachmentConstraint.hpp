#ifndef __ATTACHMENT_CONSTRAINT_HPP
#define __ATTACHMENT_CONSTRAINT_HPP

#include "constraint/Constraint.hpp"
#include "rod/XPBDRodNode.hpp"

namespace Constraint
{

class AttachmentConstraint : public XPBDConstraint<Rod::XPBDRodNode::NODE_DOF, 1>
{
    public:
    AttachmentConstraint(const Rod::XPBDRodNode* node, const AlphaVecType& alpha, const Vec3r& ref_position, const Mat3r& ref_orientation);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleNodeGradientMatType singleNodeGradient(int node_index, bool use_cache=false) const override;

    const Vec3r& referencePosition() const { return _ref_position; }
    void setReferencePosition(const Vec3r& new_pos) { _ref_position = new_pos; }
    const Mat3r& referenceOrientation() const { return _ref_orientation; }
    void setReferenceOrientation(const Mat3r& new_or) { _ref_orientation = new_or; }

    const Rod::XPBDRodNode* node() const { return _node; }

    bool operator<(const AttachmentConstraint& other) const
    {
        return _node->index < other._node->index;
    }

    private:
    const Rod::XPBDRodNode* _node;
    Vec3r _ref_position;
    Mat3r _ref_orientation;
};

} // namespace Constraint

#endif // __ATTACHMENT_CONSTRAINT_HPP