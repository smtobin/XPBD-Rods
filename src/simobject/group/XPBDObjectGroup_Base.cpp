#include "simobject/group/XPBDObjectGroup_Base.hpp"

namespace SimObject
{

XPBDObjectGroup_Base::XPBDObjectGroup_Base(const Config::XPBDObjectConfig& config)
    : XPBDObject_Base(config)
{
}

AABB XPBDObjectGroup_Base::boundingBox() const
{
    // iterate through all bodies and combine their bounding boxes
    AABB bbox;
    _objects.for_each_element([&](auto& obj) {
        AABB obj_bbox = obj.boundingBox();

        bbox.min[0] = std::min(bbox.min[0], obj_bbox.min[0]);
        bbox.min[1] = std::min(bbox.min[1], obj_bbox.min[1]);
        bbox.min[2] = std::min(bbox.min[2], obj_bbox.min[2]);

        bbox.max[0] = std::max(bbox.max[0], obj_bbox.max[0]);
        bbox.max[1] = std::max(bbox.max[1], obj_bbox.max[1]);
        bbox.max[2] = std::max(bbox.max[2], obj_bbox.max[2]);
    });

    return bbox;
}

void XPBDObjectGroup_Base::inertialUpdate(Real dt)
{
    _objects.for_each_element([&](auto& obj){
        obj.inertialUpdate(dt);
    });
}

void XPBDObjectGroup_Base::internalConstraintSolve(Real dt)
{
    _objects.for_each_element([&](auto& obj){
        obj.internalConstraintSolve(dt);
    });
}

void XPBDObjectGroup_Base::velocityUpdate(Real dt)
{
    _objects.for_each_element([&](auto& obj){
        obj.velocityUpdate(dt);
    });
}

std::vector<const OrientedParticle*> XPBDObjectGroup_Base::particles() const
{
    std::vector<const OrientedParticle*> particles_vec;
    _objects.for_each_element([&](auto& obj) {
        std::vector<const OrientedParticle*> obj_particles = obj.particles();
        particles_vec.insert(particles_vec.end(), obj_particles.begin(), obj_particles.end());
    });

    return particles_vec;
}

} // namespace SimObject