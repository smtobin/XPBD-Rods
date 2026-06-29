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
        _extractParameter("wall-length", node, _wall_length);
        _extractParameter("wall-thickness", node, _wall_thickness);
        _extractParameter("wall-height", node, _wall_height);
        _extractParameter("gap-size", node, _gap_size);
        _extractParameter("gaps", node, _gaps);

        _extractParameter("wall-color", node, _wall_color);
    }

    Real wallLength() const { return _wall_length.value; }
    Real wallThickness() const { return _wall_thickness.value; }
    Real wallHeight() const { return _wall_height.value; }
    Real gapSize() const { return _gap_size.value; }
    std::vector<bool> gaps() const { return _gaps.value; }

    Vec3r wallColor() const { return _wall_color.value; }
    
private:
    ConfigParameter<Real> _wall_length = ConfigParameter<Real>(0.1);
    ConfigParameter<Real> _wall_thickness = ConfigParameter<Real>(0.01);
    ConfigParameter<Real> _wall_height = ConfigParameter<Real>(0.02);

    ConfigParameter<std::vector<bool>> _gaps = ConfigParameter<std::vector<bool>>({0, 0, 0, 0, 0, 0});
    ConfigParameter<Real> _gap_size = ConfigParameter<Real>(0.05);

    ConfigParameter<Vec3r> _wall_color = ConfigParameter<Vec3r>(Vec3r(1.0, 1.0, 1.0));

};

} // namespace Config