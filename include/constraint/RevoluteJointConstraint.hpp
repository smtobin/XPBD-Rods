#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class RevoluteJointConstraint : public XPBDConstraint<5, 2>
{
public:
    RevoluteJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2
    );

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    const Vec3r _r1;
    const Mat3r _or1;
    
    const Vec3r _r2;
    const Mat3r _or2;

};

class OneSidedRevoluteJointConstraint : public XPBDConstraint<5, 1>
{
public:
    OneSidedRevoluteJointConstraint(
        const Vec3r& base_pos, const Mat3r& base_or,
        SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
    );

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache=false) const override;

private:
    const Vec3r _base_pos;
    const Mat3r _base_or;

    const Vec3r _r1;
    const Mat3r _or1;
};

} // namespace Constraint