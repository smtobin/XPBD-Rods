#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace SimObject
{
    class XPBDPendulum;
}

namespace Config
{

class XPBDPendulumConfig : public XPBDObjectConfig
{
public:
    using SimObjectType = SimObject::XPBDPendulum;

    explicit XPBDPendulumConfig()
        : XPBDObjectConfig()
    {}

    explicit XPBDPendulumConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
        _extractParameter("num-bodies", node, _num_bodies);
        _extractParameter("initial-angle", node, _initial_angle);
    }

    int numBodies() const { return _num_bodies.value; }
    Real initialAngle() const { return _initial_angle.value; }
    
private:
    ConfigParameter<int> _num_bodies = ConfigParameter<int>(2);
    ConfigParameter<Real> _initial_angle = ConfigParameter<Real>(45);
};

} // namespace Config