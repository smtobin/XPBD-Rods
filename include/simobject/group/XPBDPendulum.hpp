#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"

#include "config/XPBDPendulumConfig.hpp"

namespace SimObject
{

class XPBDPendulum : public XPBDObjectGroup_Base
{

public:
    XPBDPendulum(const Config::XPBDPendulumConfig& config)
        : XPBDObjectGroup_Base(config)
    {
        _num_bodies = config.numBodies();
    }

    virtual void setup() override
    {
        
        for (int i = 0; i < _num_bodies; i++)
        {
            Config::XPBDRigidBoxConfig box_config("box", Vec3r(0,0,i), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
                1000, Vec3r(0.1, 0.1, 1));
            _addObject<XPBDRigidBox>(box_config);
        }
    }

private:
    int _num_bodies;

};

} // namespace SimObject