#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"

#include "config/XPBDPendulumConfig.hpp"

#include "constraint/RevoluteJointConstraint.hpp"

namespace SimObject
{

class XPBDPendulum : public XPBDObjectGroup_Base
{

public:
    XPBDPendulum(const Config::XPBDPendulumConfig& config)
        : XPBDObjectGroup_Base(config)
    {
        _num_bodies = config.numBodies();
        _initial_angle = config.initialAngle();
    }

    virtual void setup() override;

private:
    int _num_bodies;
    Real _initial_angle;

};

} // namespace SimObject