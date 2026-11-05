#pragma once

#include "config/XPBDObjectConfig.hpp"
#include "config/RodConfig.hpp"

namespace SimObject
{
    class XPBDConcentricTubeRobot;
}

namespace Config
{

class XPBDConcentricTubeRobotConfig : public XPBDObjectConfig
{
public:
    using SimObjectType = SimObject::XPBDConcentricTubeRobot;

    explicit XPBDConcentricTubeRobotConfig()
        : XPBDObjectConfig()
    {}

    explicit XPBDConcentricTubeRobotConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
        // read config parameters for outer tube
        _outer_tube_config = RodConfig(node["outer-tube"]);
        // read config parameters for inner tube
        _inner_tube_config = RodConfig(node["inner-tube"]);

    }

    const RodConfig& outerTubeConfig() const { return _outer_tube_config; }
    const RodConfig& innerTubeConfig() const { return _inner_tube_config; }
    
private:

    RodConfig _outer_tube_config;
    RodConfig _inner_tube_config;

};

} // namespace Config