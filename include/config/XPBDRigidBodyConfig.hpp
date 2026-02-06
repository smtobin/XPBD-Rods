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
        _extractParameter("fixed", node, _fixed);
    }

    explicit XPBDRigidBodyConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity,
        Real density, bool fixed)
        : XPBDObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity)
    {
        _density.value = density;
        _fixed.value = fixed;
    }

    Real density() const { return _density.value; }
    bool fixed() const { return _fixed.value; }

private:
    ConfigParameter<Real> _density = ConfigParameter<Real>(1000);
    ConfigParameter<bool> _fixed = ConfigParameter<bool>(false);

};

} // namespace Config