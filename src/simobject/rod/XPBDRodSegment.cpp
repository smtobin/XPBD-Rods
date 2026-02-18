#include "simobject/rod/XPBDRodSegment.hpp"

#include "simobject/rod/XPBDRod.hpp"

namespace SimObject
{

XPBDRodSegment::XPBDRodSegment(XPBDRod* rod, int index1, int index2)
    : _rod(rod), _index1(index1), _index2(index2)
{
}

OrientedParticle* XPBDRodSegment::particle1()
{ 
    return &_rod->nodes().at(_index1); 
}

const OrientedParticle* XPBDRodSegment::particle1() const
{
    return &_rod->nodes().at(_index1);   
}

OrientedParticle* XPBDRodSegment::particle2()
{
    return &_rod->nodes().at(_index2);
}
const OrientedParticle* XPBDRodSegment::particle2() const
{
    return &_rod->nodes().at(_index2);
}

Real XPBDRodSegment::radius() const
{
    // YUCK! fine for now though
    return dynamic_cast<const CircleCrossSection*>(_rod->crossSection())->radius();
}

std::tuple<OrientedParticle*, OrientedParticle*, Real> XPBDRodSegment::subsegment(Real beta)
{
    if (beta >= 1)
        return std::make_tuple(&_rod->nodes().at(_index2-1), &_rod->nodes().at(_index2), 1.0);
    if (beta <= 0)
        return std::make_tuple(&_rod->nodes().at(_index1), &_rod->nodes().at(_index1+1), 0.0);

    int start_index = static_cast<int>(beta * size());
    Real new_beta = beta*size() - start_index;
    return std::make_tuple(&_rod->nodes().at(start_index+_index1), &_rod->nodes().at(start_index+_index1+1), new_beta);
} 

Vec3r XPBDRodSegment::pointOnSegment(Real beta) const
{
    if (beta >= 1)
        return particle2()->position;
    if (beta <= 0)
        return particle1()->position;

    int start_index = static_cast<int>(beta * size());
    const Vec3r p1 = _rod->nodes().at(start_index+_index1).position;
    const Vec3r p2 = _rod->nodes().at(start_index+_index1+1).position;

    Real new_beta = beta*size() - start_index;
    return (1-new_beta)*p1 + new_beta*p2; 
}


AABB XPBDRodSegment::boundingBox() const
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