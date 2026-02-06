#pragma once

#include "common/common.hpp"
#include "config/XPBDRigidBodyConfig.hpp"

namespace Config
{

class XPBDPlaneConfig : public XPBDRigidBodyConfig
{
public:
    using SimObjectType = SimObject::XPBDPlane;

    explicit XPBDPlaneConfig()
        : XPBDRigidBodyConfig()
    {}

    explicit XPBDPlaneConfig(const YAML::Node& node)
        : XPBDRigidBodyConfig(node)
    {
        _extractParameter("width", node, _width);
        _extractParameter("height", node, _height);
        _extractParameter("normal", node, _normal);
    }

    explicit XPBDPlaneConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity,
        Real density,
        Real width, Real height, const Vec3r& normal)
        : XPBDRigidBodyConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density)
    {
        _width.value = width;
        _height.value = height;
        _normal.value = normal;
    }

    Real width() const { return _width.value; }
    Real height() const { return _height.value; }
    Vec3r normal() const { return _normal.value; }

protected:
    ConfigParameter<Real> _width = ConfigParameter<Real>(1);
    ConfigParameter<Real> _height = ConfigParameter<Real>(1);
    ConfigParameter<Vec3r> _normal = ConfigParameter<Vec3r>(Vec3r(0,0,1));

};

} // namespace Config