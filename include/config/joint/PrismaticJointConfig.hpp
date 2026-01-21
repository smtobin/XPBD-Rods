#pragma once

#include "config/joint/JointConfig.hpp"

namespace Config
{

class PrismaticJointConfig : public JointConfig
{
public:
    explicit PrismaticJointConfig()
        : JointConfig()
    {}

    explicit PrismaticJointConfig(const YAML::Node& node)
        : JointConfig(node)
    {
    }
};

} // namespace Config