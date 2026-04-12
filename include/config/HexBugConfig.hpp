#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

class HexBugConfig : public XPBDObjectConfig
{
public:
    using SimObjectType = SimObject::HexBug;

    explicit HexBugConfig()
        : XPBDObjectConfig()
    {}

    explicit HexBugConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
    }
    
private:
};

} // namespace Config