#pragma once

#include "config/joint/JointConfig.hpp"

namespace Config
{

class FixedJointConfig : public JointConfig
{
public:
    explicit FixedJointConfig()
        : JointConfig()
    {}

    explicit FixedJointConfig(const YAML::Node& node)
        : JointConfig(node)
    {
    }
};

} // namespace Config