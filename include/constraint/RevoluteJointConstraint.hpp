#pragma once

#include "constraint/Constraint.hpp"

namespace Constraint
{

class RevoluteJointConstraint : public XPBDConstraint<5, 2, 0>
{
public:
    RevoluteJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2
    );

    Mat3r jointOrientation1() const { return _oriented_particles[0]->orientation * _or1; }
    Mat3r jointOrientation2() const { return _oriented_particles[1]->orientation * _or2; }

    const Vec3r& bodyJointOffset1() const { return _r1; }
    const Vec3r& bodyJointOffset2() const { return _r2; }

    const Mat3r& bodyJointOrientationOffset1() const { return _or1; }
    const Mat3r& bodyJointOrientationOffset2() const { return _or2; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Vec3r _r1;
    Mat3r _or1;
    
    Vec3r _r2;
    Mat3r _or2;

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class OneSidedRevoluteJointConstraint : public XPBDConstraint<5, 1, 0>
{
public:
    OneSidedRevoluteJointConstraint(
        const Vec3r& base_pos, const Mat3r& base_or,
        SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
    );

    Mat3r jointOrientation1() const { return _oriented_particles[0]->orientation * _or1;}
    Mat3r jointOrientation2() const { return _base_or; }

    const Vec3r& bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return Vec3r::Zero(); }

    const Mat3r& bodyJointOrientationOffset1() const { return _or1; }
    Mat3r bodyJointOrientationOffset2() const { return Mat3r::Identity(); }

    void setFixedPosition(const Vec3r& pos) { _base_pos = pos; }
    void setFixedOrientation(const Mat3r& ori) { _base_or = ori; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Vec3r _base_pos;
    Mat3r _base_or;

    Vec3r _r1;
    Mat3r _or1;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


class NormedRevoluteJointConstraint : public XPBDConstraint<2, 2, 0>
{
public:
    NormedRevoluteJointConstraint(
        SimObject::OrientedParticle* particle1, const Vec3r& r1, const Mat3r& or1,
        SimObject::OrientedParticle* particle2, const Vec3r& r2, const Mat3r& or2
    );

    Mat3r jointOrientation1() const { return _oriented_particles[0]->orientation * _or1; }
    Mat3r jointOrientation2() const { return _oriented_particles[1]->orientation * _or2; }

    const Vec3r& bodyJointOffset1() const { return _r1; }
    const Vec3r& bodyJointOffset2() const { return _r2; }

    const Mat3r& bodyJointOrientationOffset1() const { return _or1; }
    const Mat3r& bodyJointOrientationOffset2() const { return _or2; }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Vec3r _r1;
    Mat3r _or1;
    
    Vec3r _r2;
    Mat3r _or2;

};


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

class NormedOneSidedRevoluteJointConstraint : public XPBDConstraint<2, 1, 0>
{
public:
    NormedOneSidedRevoluteJointConstraint(
        const Vec3r& base_pos, const Mat3r& base_or,
        SimObject::OrientedParticle* particle, const Vec3r& joint_pos, const Mat3r& joint_or
    );

    Mat3r jointOrientation1() const { return _oriented_particles[0]->orientation * _or1;}
    Mat3r jointOrientation2() const { return _base_or; }

    const Vec3r& bodyJointOffset1() const { return _r1; }
    Vec3r bodyJointOffset2() const { return Vec3r::Zero(); }

    const Mat3r& bodyJointOrientationOffset1() const { return _or1; }
    Mat3r bodyJointOrientationOffset2() const { return Mat3r::Identity(); }

    virtual ConstraintVecType evaluate() const override;
    virtual GradientMatType gradient() const override;

private:
    Vec3r _base_pos;
    Mat3r _base_or;

    Vec3r _r1;
    Mat3r _or1;
};

} // namespace Constraint