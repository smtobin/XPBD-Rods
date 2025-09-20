#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

class XPBDRigidBodyConfig : public XPBDObjectConfig
{
public:
    explicit XPBDRigidBodyConfig()
        : XPBDObjectConfig()
    {}

    explicit XPBDRigidBodyConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
        _extractParameter("density", node, _density);
    }

    explicit XPBDRigidBodyConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity,
        Real density)
        : XPBDObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity)
    {
        _density.value = density;
    }

    Real density() const { return _density.value; }

private:
    ConfigParameter<Real> _density = ConfigParameter<Real>(1000);

};

} // namespace Config