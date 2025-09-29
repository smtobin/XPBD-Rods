#pragma once

#include "config/Config.hpp"
#include "config/ObjectRenderConfig.hpp"

namespace Config
{

class XPBDObjectConfig : public Config_Base
{
public:
    explicit XPBDObjectConfig()
        : Config_Base(), _render_config()
    {}

    explicit XPBDObjectConfig(const YAML::Node& node)
        : Config_Base(node), _render_config(node)
    {
        _extractParameter("initial-position", node, _initial_position);
        _extractParameter("initial-rotation", node, _initial_rotation);
        _extractParameter("initial-velocity", node, _initial_velocity);
        _extractParameter("initial-angular-velocity", node, _initial_angular_velocity);
    }

    explicit XPBDObjectConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity)
        : Config_Base(name), _render_config()
    {
        _initial_position.value = initial_position;
        _initial_rotation.value = initial_rotation;
        _initial_velocity.value = initial_velocity;
        _initial_angular_velocity.value = initial_angular_velocity;
    }

    const Vec3r& initialPosition() const { return _initial_position.value; }
    const Vec3r& initialRotation() const { return _initial_rotation.value; }
    const Vec3r& initialVelocity() const { return _initial_velocity.value; }
    const Vec3r& initialAngularVelocity() const { return _initial_angular_velocity.value; }

    const ObjectRenderConfig& renderConfig() const { return _render_config; }

protected:
    ConfigParameter<Vec3r> _initial_position = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));
    ConfigParameter<Vec3r> _initial_rotation = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));
    ConfigParameter<Vec3r> _initial_velocity = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));
    ConfigParameter<Vec3r> _initial_angular_velocity = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));

    ObjectRenderConfig _render_config;
};

} // namespace Config