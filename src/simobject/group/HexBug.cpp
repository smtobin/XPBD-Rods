#include "simobject/group/HexBug.hpp"

namespace SimObject
{

HexBug::HexBug(const Config::HexBugConfig& config)
    : XPBDObjectGroup_Base(config)
{

}

void HexBug::setup()
{
    Real body_length = 0.04;
    Real body_width = 0.0125;
    Real body_thickness = 0.005;

    Vec3r body_center(0,0.03,0);

    Real leg_radius = body_width/10.0;
    Real leg_length = 0.01875;

    // create "body" with just a box - 4 cm long, 1.25 cm wide, with a little thickness
    Config::XPBDRigidBoxConfig body_config(
        "hexbug_body", body_center, Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(), true,
        2000, false, Vec3r(body_width, body_thickness, body_length)
    );
    auto& body = _objects.template emplace_back<XPBDRigidBox>(body_config);

    // create legs out of 12 rods - 6 on each side of body
    for (int side = 0; side < 2; side++)
    {
        Real dx = side == 0 ? -body_width/2 + leg_radius : body_width/2 - leg_radius;
        for (int i = 0; i < 6; i++)
        {
            Vec3r pos_local = Vec3r(dx, -body_thickness/2, -body_length/2 + leg_radius + (body_length-2*leg_radius)/6*i);
            Vec3r leg_base = body_center + pos_local;
            Config::RodConfig leg_config(
                "hexbug_leg", leg_base, Vec3r(90,0,0), Vec3r(0,0,0), Vec3r(0,0,0), true,
                Config::RodElementType::LINEAR, false, false, true,
                leg_length, leg_radius*2, 10, 1000, 1e5, 0.4
            );
            auto& leg = _objects.template emplace_back<XPBDRod_<RodElement<1>>>(leg_config);
            leg.setup();

            // add fixed constraint between leg base and its initial position on the body
            // Constraint::FixedJointConstraint fixed_constraint(
            //     &leg.nodes().front(), Vec3r::Zero(), Mat3r::Identity(),
            //     &body.com(), pos_local, Math::RotMatFromXYZEulerAngles(Vec3r(90,0.1,0))
            // );
            // _constraints.push_back(std::move(fixed_constraint));
            break;
        }
        break;
        
    }

    //
}

} // namespace SimObject