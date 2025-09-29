#pragma once

#include "config/XPBDRigidBodyConfig.hpp"

namespace SimObject
{
    class XPBDRigidBox;
}

namespace Config
{

class XPBDRigidBoxConfig : public XPBDRigidBodyConfig
{
public:
    using SimObjectType = SimObject::XPBDRigidBox;

    explicit XPBDRigidBoxConfig()
        : XPBDRigidBodyConfig()
    {}

    explicit XPBDRigidBoxConfig(const YAML::Node& node)
        : XPBDRigidBodyConfig(node)
    {
        _extractParameter("size", node, _size);
    }

    explicit XPBDRigidBoxConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity,
        Real density,
        const Vec3r& size)
        : XPBDRigidBodyConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density)
    {
        _size.value = size;
    }

    Vec3r size() const { return _size.value; }

private:
    ConfigParameter<Vec3r> _size = ConfigParameter<Vec3r>(Vec3r(1.0, 1.0, 1.0));

};

} // namespace Config