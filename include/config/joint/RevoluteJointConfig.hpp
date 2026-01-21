#pragma once

#include "config/joint/JointConfig.hpp"

namespace Config
{

class RevoluteJointConfig : public JointConfig
{
public:
    explicit RevoluteJointConfig()
        : JointConfig()
    {}

    explicit RevoluteJointConfig(const YAML::Node& node)
        : JointConfig(node)
    {
    }
};

} // namespace Config