#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class FixedJointConstraint : public XPBDConstraint<6, 2>
{
    public:
    FixedJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2,
        const AlphaVecType& alpha = AlphaVecType::Zero()
    );

    Vec3r bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return _r2; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle*, bool use_cache=false) const override;

    private:
    Vec3r _r1;
    Mat3r _or1;

    Vec3r _r2;
    Mat3r _or2;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class OneSidedFixedJointConstraint : public XPBDConstraint<6, 1>
{
    public:
    OneSidedFixedJointConstraint(
        const Vec3r& ref_position, const Mat3r& ref_orientation,
        SimObject::OrientedParticle* particle, const Vec3r& r1, const Mat3r& or1,
        const AlphaVecType& alpha = AlphaVecType::Zero()
    );

    Vec3r bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return Vec3r::Zero(); }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle*, bool use_cache=false) const override;

    const Vec3r& referencePosition() const { return _ref_position; }
    void setReferencePosition(const Vec3r& new_pos) { _ref_position = new_pos; }
    const Mat3r& referenceOrientation() const { return _ref_orientation; }
    void setReferenceOrientation(const Mat3r& new_or) { _ref_orientation = new_or; }

    // bool operator<(const OneSidedFixedJointConstraint& other) const
    // {
    //     return _node->index < other._node->index;
    // }

    private:
    Vec3r _r1;
    Mat3r _or1;

    Vec3r _ref_position;
    Mat3r _ref_orientation;
};

} // namespace Constraint