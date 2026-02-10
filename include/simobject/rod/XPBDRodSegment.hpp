#pragma once

#include "simobject/OrientedParticle.hpp"
#include "simobject/AABB.hpp"

namespace SimObject
{

class XPBDRod;

class XPBDRodSegment
{
public:
    explicit XPBDRodSegment(XPBDRod* rod, int index1, int index2);

    XPBDRod* rod() { return _rod; }
    const XPBDRod* rod() const { return _rod; }

    int size() const { return _index2 - _index1; }

    int index1() const { return _index1; }
    int index2() const { return _index2; }

    std::tuple<OrientedParticle*, OrientedParticle*, Real> subsegment(Real beta); 
    Vec3r pointOnSegment(Real beta) const;

    OrientedParticle* particle1();
    const OrientedParticle* particle1() const;

    OrientedParticle* particle2();
    const OrientedParticle* particle2() const;

    Real radius() const;

    AABB boundingBox() const;
    

private:
    XPBDRod* _rod;
    int _index1;
    int _index2;
};

} // namespace SimObject