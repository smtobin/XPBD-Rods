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

    virtual void setup() override
    {
        
        Config::XPBDRigidBoxConfig base_config("base", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
            1000, Vec3r(1, 0.1, 1));
        XPBDRigidBox& base = _addObject<XPBDRigidBox>(base_config);
        base.setFixed(true);

        Real body_length = 1.0;


        std::vector<XPBDRigidBox>& bodies = _objects.template get<XPBDRigidBox>();
        for (int i = 0; i < _num_bodies; i++)
        {
            Real pos_x = (0.5 + i*body_length) * std::sin(_initial_angle * M_PI/180.0);
            Real pos_y = (-0.5 - i*body_length) * std::cos(_initial_angle * M_PI/180.0);
            Config::XPBDRigidBoxConfig box_config("box", Vec3r(pos_x,pos_y,0), Vec3r(0,0,_initial_angle), Vec3r(0,0,0), Vec3r(0,0,0),
                1000, Vec3r(0.1, body_length, 0.1));
            _addObject<XPBDRigidBox>(box_config);
        }

        _addConstraint<Constraint::RevoluteJointConstraint>(&(bodies[0].com()), Vec3r(0,0,0), Mat3r::Identity(), &(bodies[1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
        for (int i = 1; i < _num_bodies; i++)
        {
            _addConstraint<Constraint::RevoluteJointConstraint>(&(bodies[i].com()), Vec3r(0,-0.5*body_length,0), Mat3r::Identity(), &(bodies[i+1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
        }
    }

private:
    int _num_bodies;
    Real _initial_angle;

};

} // namespace SimObject