#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

class HexBugHabitatConfig : public XPBDObjectConfig
{
public:
    using SimObjectType = SimObject::HexBugHabitat;

    explicit HexBugHabitatConfig()
        : XPBDObjectConfig()
    {}

    explicit HexBugHabitatConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
        _extractParameter("wall-size", node, _wall_size);
        _extractParameter("wall-color", node, _wall_color);
    }

    Vec3r wallSize() const { return _wall_size.value; }
    Vec3r wallColor() const { return _wall_color.value; }
    
private:
    ConfigParameter<Vec3r> _wall_size = ConfigParameter<Vec3r>(Vec3r(0.01, 0.03, 0.1));
    ConfigParameter<Vec3r> _wall_color = ConfigParameter<Vec3r>(Vec3r(1.0, 1.0, 1.0));

};

} // namespace Config