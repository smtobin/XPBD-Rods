#include "simobject/group/XPBDPendulum.hpp"

namespace SimObject
{

XPBDPendulum::XPBDPendulum(const Config::XPBDPendulumConfig& config)
    : XPBDObjectGroup_Base(config)
{
    _num_bodies = config.numBodies();
    _initial_angle = config.initialAngle();
}

void XPBDPendulum::setup()
{
        
    Config::XPBDRigidBoxConfig base_config("base", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        1000, true, Vec3r(1, 0.1, 1));
    XPBDRigidBox& base = _addObject<XPBDRigidBox>(base_config);

    Real body_length = 1.0;


    std::vector<XPBDRigidBox>& bodies = _objects.template get<XPBDRigidBox>();
    for (int i = 0; i < _num_bodies; i++)
    {
        Real pos_x = (0.5 + i*body_length) * std::sin(_initial_angle * M_PI/180.0);
        Real pos_y = (-0.5 - i*body_length) * std::cos(_initial_angle * M_PI/180.0);
        Config::XPBDRigidBoxConfig box_config("box", Vec3r(pos_x,pos_y,0), Vec3r(0,10,_initial_angle), Vec3r(0,0,0), Vec3r(0,0,0),
            1000, false, Vec3r(0.1, body_length, 0.1));
        _addObject<XPBDRigidBox>(box_config);
    }
    _addConstraint<Constraint::OneSidedRevoluteJointConstraint>(bodies[0].com().position, bodies[0].com().orientation, &(bodies[1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
    // _addConstraint<Constraint::NormedOneSidedRevoluteJointConstraint>(bodies[0].com().position, bodies[0].com().orientation, &(bodies[1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
    for (int i = 1; i < _num_bodies; i++)
    {
        _addConstraint<Constraint::RevoluteJointConstraint>(&(bodies[i].com()), Vec3r(0,-0.5*body_length,0), Mat3r::Identity(), &(bodies[i+1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
        // _addConstraint<Constraint::NormedRevoluteJointConstraint>(&(bodies[i].com()), Vec3r(0,-0.5*body_length,0), Mat3r::Identity(), &(bodies[i+1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
    }
}

} // namespace SimObject