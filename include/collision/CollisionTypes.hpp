#pragma once

#include "common/common.hpp"
#include "simobject/OrientedParticle.hpp"
#include "simobject/rod/RodCollisionSegment.hpp"

#include <variant>

namespace Collision
{

struct RigidRigidCollision
{
    SimObject::XPBDRigidBody_Base* rb1;
    SimObject::XPBDRigidBody_Base* rb2;
    // SimObject::OrientedParticle* particle1; // rigid body particle (COM) 1
    // SimObject::OrientedParticle* particle2; // rigid body particle (COM) 2
    Vec3r cp_local1;    // contact point in local rigid body 1 frame
    Vec3r cp_local2;    // contact point in local rigid body 2 frame
    Vec3r normal;       // collision normal (points from body 1 to body 2)
};

struct RigidSegmentCollision
{
    SimObject::XPBDRigidBody_Base* rb;  // the rigid body in collision
    Vec3r cp_local_rb;  // contact point in local rigid body frame
    SimObject::XPBDRod* rod;    // the rod in collision
    SimObject::OrientedParticle* segment_particle1; // one endpoint of rod segment
    SimObject::OrientedParticle* segment_particle2; // other endpoint of rod segment
    Real beta; // interpolation parameter in [0,1] 
    Vec3r cp_local_rod;  // contact point in local interpolated rod frame
    Vec3r normal;   //collision normal (points outward from segment)
};

struct SegmentSegmentCollision
{
    SimObject::RodElement_Base* element1; // element on rod 1
    Real s_hat1; // interpolation parameter in [0,1] for 1st segment
    Vec3r cp_local1; // contact point in the interpolated coordinate frame
    SimObject::RodElement_Base* element2; // element on rod 2
    Real s_hat2; // interpolation parameter in [0,1] 
    Vec3r cp_local2; // contact point in the interpolated coordinate frame
    Vec3r normal;   //collision normal (points outward from segment)
    Real mu_s1;
    Real mu_d1;
    Real mu_s2;
    Real mu_d2;
};

using DetectedCollision = std::variant<RigidRigidCollision, RigidSegmentCollision, SegmentSegmentCollision>;

} // namespace Collision