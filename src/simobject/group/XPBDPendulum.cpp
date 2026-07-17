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
        
    Config::XPBDRigidBoxConfig base_config("base", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false, 0.2, 0.1,
        1000, true, Vec3r(1, 0.1, 1));
    Config::MeshRenderConfig base_mesh_config(
        Config::ObjectRenderConfig::RenderType::PBR,
        "../resource/meshes/pendulum_base.stl",
        std::nullopt, std::nullopt, std::nullopt,
        0, 0.3, 1.0, Vec3r(1.0,1.0, 1.0),
        false,
        true, false,
        Vec3r(0,0.3,0), Vec3r(0,0,0), Vec3r::Ones()
    );
    base_config.addRenderMeshConfig(base_mesh_config);
    _addObject<XPBDRigidBox>(base_config);

    Real body_length = 1.0;


    std::vector<XPBDRigidBox>& bodies = _objects.template get<XPBDRigidBox>();
    for (int i = 0; i < _num_bodies; i++)
    {
        Real pos_x = (0.5 + i*body_length) * std::sin(_initial_angle * M_PI/180.0);
        Real pos_y = (-0.5 - i*body_length) * std::cos(_initial_angle * M_PI/180.0);
        Config::XPBDRigidBoxConfig box_config("box", Vec3r(pos_x,pos_y,0), Vec3r(0,0,_initial_angle), Vec3r(0,0,0), Vec3r(0,0,0), false, 0.2, 0.1,
            1000, false, Vec3r(0.1, body_length, 0.1));

        std::string mesh_filename = (i%2 == 0) ? "../resource/meshes/pendulum_male.stl" : "../resource/meshes/pendulum_female.stl";
        if (i==_num_bodies-1)
            mesh_filename = "../resource/meshes/pendulum_tip.stl";

        Vec3r offset = (i%2 == 0) ? Vec3r(0, 0, -0.1) : Vec3r(0,0,0);
        Vec3r color = (i%2 == 0) ? Vec3r(1, 0.5, 0) : Vec3r(1.0, 1.0, 1.0);
        Config::MeshRenderConfig link_mesh_config(
            Config::ObjectRenderConfig::RenderType::PBR,
            mesh_filename,
            std::nullopt, std::nullopt, std::nullopt,
            0, 0.3, 1.0, color,
            false,
            true, false,
            offset, Vec3r(0,0,0), Vec3r::Ones()
        );
        box_config.addRenderMeshConfig(link_mesh_config);
        _addObject<XPBDRigidBox>(box_config);
    }
    _addConstraint<Constraint::OneSidedRevoluteJointConstraint>(bodies[0].com().position, bodies[0].com().orientation, &(bodies[1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
    // _addConstraint<Constraint::NormedOneSidedRevoluteJointConstraint>(bodies[0].com().position, bodies[0].com().orientation, &(bodies[1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
    // _addConstraint<Constraint::NormedRevoluteJointConstraint>(&(bodies[0].com()), Vec3r::Zero(), Mat3r::Identity(), &(bodies[1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
    for (int i = 1; i < _num_bodies; i++)
    {
        _addConstraint<Constraint::RevoluteJointConstraint>(&(bodies[i].com()), Vec3r(0,-0.5*body_length,0), Mat3r::Identity(), &(bodies[i+1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
        // _addConstraint<Constraint::NormedRevoluteJointConstraint>(&(bodies[i].com()), Vec3r(0,-0.5*body_length,0), Mat3r::Identity(), &(bodies[i+1].com()), Vec3r(0,0.5*body_length,0), Mat3r::Identity());
    }
}

} // namespace SimObject