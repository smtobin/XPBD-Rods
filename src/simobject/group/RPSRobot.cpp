#include "simobject/group/RPSRobot.hpp"

namespace SimObject
{

RPSRobot::RPSRobot(const Config::RPSRobotConfig& config)
    : XPBDObjectGroup_Base(config)
{

}

void RPSRobot::setup()
{
    // reserve space for rigid boxes
    _objects.template reserve<XPBDRigidBox>(15);

    // create bottom platform
    Real base_radius = 0.25;
    Vec3r base_position(0,1,0);
    Vec3r base_size(2*base_radius, 0.05, 2*base_radius);
    Config::XPBDRigidBoxConfig base_config(
        "rps_base", base_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        false, 1000, true, base_size
    );
    auto& base = _objects.template emplace_back<XPBDRigidBox>(base_config);
    base.setup();

    // create top platform
    Real top_radius = 0.15;
    Real robot_height = 0.5;
    Vec3r top_position = base_position + Vec3r(0, robot_height, 0);
    Vec3r top_size(2*top_radius, 0.05, 2*top_radius);
    Config::XPBDRigidBoxConfig top_config(
        "rps_top", top_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        false, 1000, false, top_size
    );
    auto& top = _objects.template emplace_back<XPBDRigidBox>(top_config);
    top.setup();

    // create 3 R-P-S linkages connecting bottom to top
    Real angles[] = {0, 120.0, 240.0};

    // first link in the chain
    XPBDRigidBox* first_links[3];
    XPBDRigidBox* second_links[3];
    for (int i = 0; i < 3; i++)
    {
        // given the length of the first link, the top and bottom radii, and the height of the robot, find the angle that the next two links need to be at
        Vec3r first_link_base_pos = base_position + Vec3r(base_radius*std::sin(angles[i]*M_PI/180.0), 0, base_radius*std::cos(angles[i]*M_PI/180.0));
        Vec3r second_link_end_pos(
            (top_position[0]+top_radius)*std::sin(angles[i]*M_PI/180.0),
            top_position[1],
            (top_position[2]+top_radius)*std::cos(angles[i]*M_PI/180.0)
        );
        
        // find necessary angle
        Vec3r v1 = first_link_base_pos - base_position;
        Vec3r v2 = second_link_end_pos - first_link_base_pos;
        Real angle = std::acos(std::abs(v1.dot(v2)) / (v1.norm()*v2.norm()));

        Mat3r first_link_rot_mat = Math::RotMatFromXYZEulerAngles(Vec3r(0,angles[i],0)) * Math::RotMatFromXYZEulerAngles(Vec3r(angle*180/M_PI,0,0));
        Vec3r first_link_rot = Math::XYZEulerAnglesFromRotMat(first_link_rot_mat);
        Real first_link_length = v2.norm()/2;
        Vec3r first_link_pos = first_link_base_pos + v2/4;
        Config::XPBDRigidBoxConfig first_link_config(
            "rps_chain" + std::to_string(i) + "_first",
            first_link_pos, first_link_rot, Vec3r(0,0,0), Vec3r(0,0,0),
            false, 1000, false, Vec3r(0.05, 0.05, first_link_length)
        );
        first_links[i] = &_objects.template emplace_back<XPBDRigidBox>(first_link_config);
        first_links[i]->setup();

        // create second link
        // Vec3r first_link_end_pos = first_link_pos + Vec3r((first_link_length/2)*std::sin(angles[i]*M_PI/180.0), 0, (first_link_length/2)*std::cos(angles[i]*M_PI/180.0));
        Vec3r second_link_pos = second_link_end_pos - v2/4;
        Real second_link_length = first_link_length;

        Vec3r second_link_rot = first_link_rot;
        Config::XPBDRigidBoxConfig second_link_config(
            "rps_chain" + std::to_string(i) + "_second",
            second_link_pos, second_link_rot, Vec3r(0,0,0), Vec3r(0,0,0),
            false, 1000, false, Vec3r(0.05, 0.05, second_link_length)
        );
        second_links[i] = &_objects.template emplace_back<XPBDRigidBox>(second_link_config);
        second_links[i]->setup();


        /** Joints */
        Mat3r base_rev_joint_rot_offset = Math::RotMatFromXYZEulerAngles(first_link_rot) * Math::RotMatFromXYZEulerAngles(Vec3r(0,-90,0));
        Constraint::RevoluteJointConstraint revolute_constraint(
            &first_links[i]->com(), Vec3r(0,0,first_link_length/2), Math::RotMatFromXYZEulerAngles(Vec3r(0,-90,0)),
            &base.com(), Vec3r(base_radius*std::sin(angles[i]*M_PI/180.0), 0, base_radius*std::cos(angles[i]*M_PI/180.0)), base_rev_joint_rot_offset
        );
        _constraints.template push_back<Constraint::RevoluteJointConstraint>(std::move(revolute_constraint));

        Constraint::PrismaticJointConstraint prismatic_constraint(
            &first_links[i]->com(), Vec3r(0,0,-first_link_length/2), Mat3r::Identity(),
            &second_links[i]->com(), Vec3r(0,0,second_link_length/2), Mat3r::Identity()
        );
        Constraint::PrismaticJointLimitConstraint prismatic_joint_limit_constraint(
            prismatic_constraint, -0.1, 0.1
        );
        _constraints.template push_back<Constraint::PrismaticJointConstraint>(std::move(prismatic_constraint));
        _constraints.template push_back<Constraint::PrismaticJointLimitConstraint>(std::move(prismatic_joint_limit_constraint));

        Constraint::SphericalJointConstraint spherical_constraint(
            &second_links[i]->com(), Vec3r(0,0,-second_link_length/2), Mat3r::Identity(),
            &top.com(), Vec3r(top_radius*std::sin(angles[i]*M_PI/180.0), 0, top_radius*std::cos(angles[i]*M_PI/180.0)), Mat3r::Identity()
        );
        _constraints.template push_back<Constraint::SphericalJointConstraint>(std::move(spherical_constraint));

        
    }


    
}

} // namespace SimObject