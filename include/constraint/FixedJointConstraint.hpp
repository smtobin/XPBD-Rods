#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class FixedJointConstraint : public XPBDConstraint<SimObject::OrientedParticle::DOF, 1>
{
    public:
    FixedJointConstraint(SimObject::OrientedParticle* node, const AlphaVecType& alpha, const Vec3r& ref_position, const Mat3r& ref_orientation);

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle*, bool use_cache=false) const override;

    const Vec3r& referencePosition() const { return _ref_position; }
    void setReferencePosition(const Vec3r& new_pos) { _ref_position = new_pos; }
    const Mat3r& referenceOrientation() const { return _ref_orientation; }
    void setReferenceOrientation(const Mat3r& new_or) { _ref_orientation = new_or; }

    // bool operator<(const FixedJointConstraint& other) const
    // {
    //     return _node->index < other._node->index;
    // }

    private:
    Vec3r _ref_position;
    Mat3r _ref_orientation;
};

} // namespace Constraint