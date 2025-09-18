#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

class XPBDRigidBodyConfig : public XPBDObjectConfig
{
public:
    explicit XPBDRigidBodyConfig()
        : XPBDObjectConfig()
    {}

    explicit XPBDRigidBodyConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {

    }

    explicit XPBDRigidBodyConfig(const std::string& name)
        : XPBDObjectConfig(name)
    {

    }

};

} // namespace Config