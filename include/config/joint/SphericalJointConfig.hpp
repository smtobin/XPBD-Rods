#pragma once

#include "config/joint/JointConfig.hpp"

namespace Config
{

class SphericalJointConfig : public JointConfig
{
public:
    explicit SphericalJointConfig()
        : JointConfig()
    {}

    explicit SphericalJointConfig(const YAML::Node& node)
        : JointConfig(node)
    {
    }
};

} // namespace Config