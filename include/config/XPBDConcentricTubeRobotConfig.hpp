#pragma once

#include "config/XPBDObjectConfig.hpp"

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
    }
    
private:
};

} // namespace Config