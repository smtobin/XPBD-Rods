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

    Mat3r jointOrientation1() const { return _particles[0]->orientation * _or1; }
    Mat3r jointOrientation2() const { return _particles[1]->orientation * _or2; }

    Vec3r bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return _r2; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    const Vec3r _r1;
    const Mat3r _or1;
    
    const Vec3r _r2;
    const Mat3r _or2;

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class OneSidedRevoluteJointConstraint : public XPBDConstraint<5, 1>
{
public:
    OneSidedRevoluteJointConstraint(
        const Vec3r& base_pos, const Mat3r& base_or,
        SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
    );

    Mat3r jointOrientation1() const { return _particles[0]->orientation * _or1;}
    Mat3r jointOrientation2() const { return _base_or; }

    Vec3r bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return Vec3r::Zero(); }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache=false) const override;

private:
    const Vec3r _base_pos;
    const Mat3r _base_or;

    const Vec3r _r1;
    const Mat3r _or1;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


class NormedRevoluteJointConstraint : public XPBDConstraint<2, 2>
{
public:
    NormedRevoluteJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2
    );

    Mat3r jointOrientation1() const { return _particles[0]->orientation * _or1; }
    Mat3r jointOrientation2() const { return _particles[1]->orientation * _or2; }

    Vec3r bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return _r2; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient(bool update_cache=true) const override;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* node_ptr, bool use_cache=false) const override;

private:
    const Vec3r _r1;
    const Mat3r _or1;
    
    const Vec3r _r2;
    const Mat3r _or2;

};


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

class NormedOneSidedRevoluteJointConstraint : public XPBDConstraint<2, 1>
{
public:
    NormedOneSidedRevoluteJointConstraint(
        const Vec3r& base_pos, const Mat3r& base_or,
        SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
    );

    Mat3r jointOrientation1() const { return _particles[0]->orientation * _or1;}
    Mat3r jointOrientation2() const { return _base_or; }

    Vec3r bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return Vec3r::Zero(); }

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