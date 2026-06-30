#include "collision/CollisionObject.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rod/RodCollisionSegment.hpp"
#include "simobject/rigidbody/XPBDPlane.hpp"

namespace Collision
{

CollisionObject::CollisionObject(SimObject::XPBDRigidSphere* sphere)
    : obj(sphere), type(ColliderType::Sphere), fixed(sphere->com().fixed)
{}

CollisionObject::CollisionObject(SimObject::XPBDRigidBox* box)
    : obj(box), type(ColliderType::Box), fixed(box->com().fixed)
{}

CollisionObject::CollisionObject(SimObject::RodCollisionSegment* segment)
    : obj(segment), type(ColliderType::RodSegment), fixed(false)
{}

CollisionObject::CollisionObject(SimObject::XPBDPlane* plane)
    : obj(plane), type(ColliderType::Plane), fixed(plane->com().fixed)
{}

SimObject::AABB CollisionObject::boundingBox() const
{
    
    if (type == ColliderType::Sphere)
    {
        SimObject::XPBDRigidSphere* sphere = static_cast<SimObject::XPBDRigidSphere*>(obj);
        SimObject::AABB bbox = sphere->boundingBox();
        Vec3r center = (bbox.max + bbox.min)/2;
        Vec3r halfsize = (bbox.max - bbox.min)/2;
        halfsize[0] += 2*std::abs(sphere->com().lin_velocity[0]) * COLLISION_CHECK_INTERVAL;
        halfsize[1] += 2*std::abs(sphere->com().lin_velocity[1]) * COLLISION_CHECK_INTERVAL;
        halfsize[2] += 2*std::abs(sphere->com().lin_velocity[2]) * COLLISION_CHECK_INTERVAL;

        bbox.max = center + halfsize;
        bbox.min = center - halfsize;
        return bbox;
    }
    else if (type == ColliderType::Box)
    {
        SimObject::XPBDRigidBox* box = static_cast<SimObject::XPBDRigidBox*>(obj);
        SimObject::AABB bbox = box->boundingBox();
        Vec3r center = (bbox.max + bbox.min)/2;
        Vec3r halfsize = (bbox.max - bbox.min)/2;
        halfsize[0] += 2*std::abs(box->com().lin_velocity[0]) * COLLISION_CHECK_INTERVAL;
        halfsize[1] += 2*std::abs(box->com().lin_velocity[1]) * COLLISION_CHECK_INTERVAL;
        halfsize[2] += 2*std::abs(box->com().lin_velocity[2]) * COLLISION_CHECK_INTERVAL;

        bbox.max = center + halfsize;
        bbox.min = center - halfsize;
        return bbox;
    }
    else if (type == ColliderType::RodSegment)
    {
        SimObject::RodCollisionSegment* segment = static_cast<SimObject::RodCollisionSegment*>(obj);
        return segment->boundingBox();
    }
    else
    {
        throw std::runtime_error("CollisionObject::boundingBox object type unknown!");
        return SimObject::AABB();
    }
}

} // namespace Collision