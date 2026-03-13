#include "simobject/rod/RodCollisionSegment.hpp"

#include "simobject/rod/XPBDRod.hpp"

namespace SimObject
{

RodCollisionSegment::RodCollisionSegment(const std::vector<RodElement_Base*> elements, Real radius, Real mu_s, Real mu_d)
    : _elements(elements), _radius(radius), _mu_s(mu_s), _mu_d(mu_d)
{
}

AABB RodCollisionSegment::boundingBox() const
{
    const Vec3r& p1_pos = particle1()->position;
    const Vec3r& p2_pos = particle2()->position; 
    AABB bbox;
    bbox.min[0] = std::min(p1_pos[0], p2_pos[0]) - radius();
    bbox.min[1] = std::min(p1_pos[1], p2_pos[1]) - radius();
    bbox.min[2] = std::min(p1_pos[2], p2_pos[2]) - radius();

    bbox.max[0] = std::max(p1_pos[0], p2_pos[0]) + radius();
    bbox.max[1] = std::max(p1_pos[1], p2_pos[1]) + radius();
    bbox.max[2] = std::max(p1_pos[2], p2_pos[2]) + radius();

    return bbox;
}

} // namespce SimObject