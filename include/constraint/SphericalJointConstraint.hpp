#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class SphericalJointConstraint : public XPBDConstraint<3, 2>
{
public:
    SphericalJointConstraint(
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

class OneSidedSphericalJointConstraint : public XPBDConstraint<3, 1>
{
public:
    OneSidedSphericalJointConstraint(
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


class NormedSphericalJointConstraint : public XPBDConstraint<1, 2>
{
public:
    NormedSphericalJointConstraint(
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

class NormedOneSidedSphericalJointConstraint : public XPBDConstraint<1, 1>
{
public:
    NormedOneSidedSphericalJointConstraint(
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