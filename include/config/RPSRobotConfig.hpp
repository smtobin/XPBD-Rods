#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

class RPSRobotConfig : public XPBDObjectConfig
{
public:
    using SimObjectType = SimObject::RPSRobot;

    explicit RPSRobotConfig()
        : XPBDObjectConfig()
    {}

    explicit RPSRobotConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
    }
    
private:

};

} // namespace Config
