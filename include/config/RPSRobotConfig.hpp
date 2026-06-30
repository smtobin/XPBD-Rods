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
        _extractParameter("use-normed-constraints", node, _normed_constraints);
    }

    bool normedConstraints() const { return _normed_constraints.value; }
    
private:
    ConfigParameter<bool> _normed_constraints = ConfigParameter<bool>(false);
};

} // namespace Config
