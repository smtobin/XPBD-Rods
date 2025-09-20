#pragma once

#include "config/XPBDRigidBodyConfig.hpp"

namespace Config
{

class XPBDRigidSphereConfig : public XPBDRigidBodyConfig
{
public:
    explicit XPBDRigidSphereConfig()
        : XPBDRigidBodyConfig()
    {}

    explicit XPBDRigidSphereConfig(const YAML::Node& node)
        : XPBDRigidBodyConfig(node)
    {
        _extractParameter("radius", node, _radius);
    }

    explicit XPBDRigidSphereConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity,
        Real density,
        Real radius)
        : XPBDRigidObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density)
    {
        _radius.value = radius;
    }

    Real radius() const { return _radius.value; }

private:
    ConfigParameter<Real> _radius = ConfigParameter<Real>(0.5);

};

} // namespace Config