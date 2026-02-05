#pragma once

#include "common/common.hpp"
#include "simobject/OrientedParticle.hpp"

#include <variant>

namespace Collision
{

struct RigidRigidCollision
{
    SimObject::OrientedParticle* particle1; // rigid body particle (COM) 1
    SimObject::OrientedParticle* particle2; // rigid body particle (COM) 2
    Vec3r cp_local1;    // contact point in local rigid body 1 frame
    Vec3r cp_local2;    // contact point in local rigid body 2 frame
    Vec3r normal;       // collision normal (points from body 1 to body 2)
};

struct RigidSegmentCollision
{
    SimObject::OrientedParticle* rb_particle;   // rigid body particle (COM)
    Vec3r rb_cp_local;  // contact point in local rigid body frame
    SimObject::OrientedParticle* segment_particle1; // one endpoint of rod segment
    SimObject::OrientedParticle* segment_particle2; // other endpoint of rod segment
    Real alpha; // interpolation parameter in [0,1] 
    Real radius;  // cross-section radius. TODO: generalize to generic cross-sections
    Vec3r normal;   //collision normal (points outward from segment)
};

struct SegmentSegmentCollision
{
    SimObject::OrientedParticle* segment1_particle1; // one endpoint of rod segment 1
    SimObject::OrientedParticle* segment1_particle2; // other endpoint of rod segment 1
    Real alpha1; // interpolation parameter in [0,1] for 1st segment
    Real radius1;  // cross-section radius for 1st segment. TODO: generalize to generic cross-sections
    SimObject::OrientedParticle* segment2_particle1; // one endpoint of rod segment 2
    SimObject::OrientedParticle* segment2_particle2; // other endpoint of rod segment 2
    Real alpha2; // interpolation parameter in [0,1] 
    Real radius2;  // cross-section radius. TODO: generalize to generic cross-sections
    Vec3r normal;   //collision normal (points outward from segment)
};

using DetectedCollision = std::variant<RigidRigidCollision, RigidSegmentCollision, SegmentSegmentCollision>;

} // namespace Collision