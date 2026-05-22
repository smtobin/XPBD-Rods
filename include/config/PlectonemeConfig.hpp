#pragma once

#include "config/XPBDObjectConfig.hpp"

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
        : XPBDObjectConfig(node)
    {
        _extractParameter("revolutions", node, _revolutions);
    }

    Real revolutions() const { return _revolutions.value; }
    
private:
    ConfigParameter<Real> _revolutions = ConfigParameter<Real>(2);
};

} // namespace Config
