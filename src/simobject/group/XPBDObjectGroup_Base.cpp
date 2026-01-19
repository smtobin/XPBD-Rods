#include "simobject/group/XPBDObjectGroup_Base.hpp"

namespace SimObject
{

XPBDObjectGroup_Base::XPBDObjectGroup_Base(const Config::XPBDObjectConfig& config)
    : XPBDObject_Base(config)
{
}

XPBDObjectGroup_Base::~XPBDObjectGroup_Base() = default;
XPBDObjectGroup_Base::XPBDObjectGroup_Base(XPBDObjectGroup_Base&&) noexcept = default;
XPBDObjectGroup_Base& XPBDObjectGroup_Base::operator=(XPBDObjectGroup_Base&&) noexcept = default;

const XPBDObjects_Container& XPBDObjectGroup_Base::objects() const { return _objects; }
const XPBDConstraints_Container& XPBDObjectGroup_Base::constraints() const { return _constraints; }

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