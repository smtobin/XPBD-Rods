#include "collision/CollisionObject.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rod/XPBDRodSegment.hpp"

namespace Collision
{

SimObject::AABB CollisionObject::boundingBox() const
{
    if (type == ColliderType::Sphere)
    {
        SimObject::XPBDRigidSphere* sphere = static_cast<SimObject::XPBDRigidSphere*>(obj);
        return sphere->boundingBox();
    }
    else if (type == ColliderType::Box)
    {
        SimObject::XPBDRigidBox* box = static_cast<SimObject::XPBDRigidBox*>(obj);
        return box->boundingBox();
    }
    else if (type == ColliderType::RodSegment)
    {
        SimObject::XPBDRodSegment* segment = static_cast<SimObject::XPBDRodSegment*>(obj);
        return segment->boundingBox();
    }
    else
    {
        throw std::runtime_error("CollisionObject::boundingBox object type unknown!");
        return SimObject::AABB();
    }
}

} // namespace Collision