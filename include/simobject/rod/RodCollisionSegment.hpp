#pragma once

#include "simobject/OrientedParticle.hpp"
#include "simobject/AABB.hpp"
#include "simobject/rod/RodElement.hpp"

namespace SimObject
{

class RodCollisionSegment
{
public:
    RodCollisionSegment() = default;
    explicit RodCollisionSegment(const std::vector<RodElement_Base*> elements, Real radius, Real mu_s, Real mu_d);

    const std::vector<RodElement_Base*> elements() const { return _elements; }

    OrientedParticle* particle1() const { return _elements.front()->firstNode(); }
    OrientedParticle* particle2() const { return _elements.back()->lastNode(); }

    Real radius() const { return _radius; }
    Real staticFrictionCoeff() const { return _mu_s; }
    Real dynamicFrictionCoeff() const { return _mu_d; }
    AABB boundingBox() const;
    

private:
    std::vector<RodElement_Base*> _elements;
    Real _radius;

    Real _mu_s;
    Real _mu_d;
};

} // namespace SimObject