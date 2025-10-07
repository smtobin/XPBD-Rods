#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"

namespace SimObject
{

class XPBDFourBarLinkage : public XPBDObjectGroup_Base
{

public:
    XPBDFourBarLinkage(const Config::XPBDPendulumConfig& config)
        : XPBDObjectGroup_Base(config)
    {
        _num_bodies = config.numBodies();
    }

    virtual void setup() override
    {
        
        // Config::XPBDRigidBoxConfig base_config("base", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        //     1000, Vec3r(1, 0.1, 1));
        // XPBDRigidBox& base = _addObject<XPBDRigidBox>(base_config);
        // base.com().fixed = true;


        // std::vector<XPBDRigidBox>& bodies = _objects.template get<XPBDRigidBox>();
        // for (int i = 0; i < _num_bodies; i++)
        // {
        //     Config::XPBDRigidBoxConfig box_config("box", Vec3r(0,i+0.5,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        //         1000, Vec3r(0.1, 1, 0.1));
        //     _addObject<XPBDRigidBox>(box_config);
        //     _addConstraint<Constraint::RevoluteJointConstraint>(&bodies[i], Vec3r(0,0.5,0), Mat3r::Identity(), &bodies[i+1], Vec3r(0,-0.5,0), Mat3r::Identity());
        // }
    }

private:
    int _num_bodies;

};

} // namespace SimObject