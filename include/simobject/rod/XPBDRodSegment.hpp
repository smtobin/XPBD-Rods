#pragma once

#include "simobject/OrientedParticle.hpp"

namespace SimObject
{

class XPBDRodSegment
{
public:
    explicit XPBDRodSegment(OrientedParticle* particle1, OrientedParticle* particle2, Real radius)
        : _particle1(particle1), _particle2(particle2), _radius(radius)
    {}

    OrientedParticle* particle1() { return _particle1; }
    const OrientedParticle* particle1() const { return _particle1; }

    OrientedParticle* particle2() { return _particle2; }
    const OrientedParticle* particle2() const { return _particle2; }

    Real radius() const { return _radius; }

    AABB boundingBox() const
    {
        const Vec3r& p1_pos = _particle1->position;
        const Vec3r& p2_pos = _particle2->position; 
        AABB bbox;
        bbox.min[0] = std::min(p1_pos[0], p2_pos[0]);
        bbox.min[1] = std::min(p1_pos[1], p2_pos[1]);
        bbox.min[2] = std::min(p1_pos[2], p2_pos[2]);

        bbox.max[0] = std::max(p1_pos[0], p2_pos[0]);
        bbox.max[1] = std::max(p1_pos[1], p2_pos[1]);
        bbox.max[2] = std::max(p1_pos[2], p2_pos[2]);

        return bbox;
    }

private:
    OrientedParticle* _particle1;
    OrientedParticle* _particle2;

    /** TODO: adapt for non-circular cross-sections */
    Real _radius;
};

} // namespace SimObject