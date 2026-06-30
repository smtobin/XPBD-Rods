#pragma once

#include "config/XPBDObjectConfig.hpp"
#include "config/RodConfig.hpp"

namespace Config
{

class PlectonemeConfig : public XPBDObjectConfig
{
public:
    using SimObjectType = SimObject::Plectoneme;

    explicit PlectonemeConfig()
        : XPBDObjectConfig()
    {}

    explicit PlectonemeConfig(const YAML::Node& node)
        : XPBDObjectConfig(node), _rod_config(node)
    {
        _extractParameter("revolutions", node, _revolutions);
    }

    Real revolutions() const { return _revolutions.value; }

    const RodConfig& rodConfig() const { return _rod_config; }
    
private:
    ConfigParameter<Real> _revolutions = ConfigParameter<Real>(2);

    RodConfig _rod_config;
};

} // namespace Config
