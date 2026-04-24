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
    Real base_radius = 0.5;
    Vec3r base_position(0,1,0);
    Vec3r base_size(base_radius, 0.05, base_radius);
    Config::XPBDRigidBoxConfig base_config(
        "rps_base", base_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        false, 1000, true, base_size
    );
    auto& base = _objects.template emplace_back<XPBDRigidBox>(base_config);

    // create top platform
    Vec3r top_position(0,2,0);
    Vec3r top_size(0.3, 0.05, 0.3);
    Config::XPBDRigidBoxConfig top_config(
        "rps_top", top_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        false, 1000, false, top_size
    );
    auto& top = _objects.template emplace_back<XPBDRigidBox>(top_config);

    // create 3 R-P-S linkages connecting bottom to top
    Real angles[] = {0, 120.0, 240.0};
    Real first_link_length = 0.25;
    for (int i = 0; i < 3; i++)
    {
        Vec3r first_link_rot(0, angles[i], 0);
        Vec3r first_link_pos(
            (base_radius+first_link_length/2)*std::sin(angles[i]*M_PI/180.0),
             base_position[1],
            (base_radius+first_link_length/2)*std::cos(angles[i]*M_PI/180.0)
        );
        Config::XPBDRigidBoxConfig first_link_config(
            "rps_chain" + std::to_string(i) + "_first",
            first_link_pos, first_link_rot, Vec3r(0,0,0), Vec3r(0,0,0),
            false, 1000, false, Vec3r(0.05, 0.05, first_link_length)
        );
        _objects.template emplace_back<XPBDRigidBox>(first_link_config);
    }


    
}

} // namespace SimObject