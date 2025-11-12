#include "simobject/group/XPBDConcentricTubeRobot.hpp"

#include "config/RodConfig.hpp"

#include "constraint/PointLineConstraint.hpp"

namespace SimObject
{

XPBDConcentricTubeRobot::XPBDConcentricTubeRobot(const Config::XPBDConcentricTubeRobotConfig& config)
    : XPBDObjectGroup_Base(config)
{
    _base_position = config.initialPosition();
    _base_orientation = Math::RotMatFromXYZEulerAngles(config.initialRotation());
}

void XPBDConcentricTubeRobot::setup()
{
    Config::RodConfig outer_rod_config(
        "outer", _base_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), true, false,
        1, 0.1, 10, 1000, 1e6, 0.3
    );

    SimObject::CircleCrossSection outer_cross_section(0.1/2.0, 10);

    Config::RodConfig inner_rod_config(
        "inner", _base_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), true, false,
        1.5, 0.08, 10, 1000, 1e6, 0.3
    );

    SimObject::CircleCrossSection inner_cross_section(0.08/2.0, 10);

    _outer_rod = &_addObject<XPBDRod>(outer_rod_config, outer_cross_section);
    _inner_rod = &_addObject<XPBDRod>(inner_rod_config, inner_cross_section);

    // create the initial point-on-line constraints
    _updateConcentricityConstraints();
    
}

void XPBDConcentricTubeRobot::_updateConcentricityConstraints()
{
    // for now, clear the point-on-line constraints every time
    _constraints.template clear<Constraint::PointLineConstraint>();

    std::vector<OrientedParticle>& outer_nodes = _outer_rod->nodes();
    std::vector<OrientedParticle>& inner_nodes = _inner_rod->nodes();

    // go through each inner node and project it onto a line segment in the outer tube
    for (const auto& inner_node : inner_nodes)
    {
        // project onto the first segment of the outer tube - if t is negative we are outside of the outer tube
        Real t1 = Math::projectPointOntoLine(inner_node.position, outer_nodes[0].position, outer_nodes[1].position);
        if (t1 < 0)
            continue;

        // project onto the last segment of the outer tube - if t is >1 we are outside of the outer tube
        Real t2 = Math::projectPointOntoLine(inner_node.position, outer_nodes[outer_nodes.size()-2].position, outer_nodes.back().position);
        if (t2 > 1)
            continue;

        // this inner node is somewhere inside the outer tube - now we need to find which segment
        Real t = t1;
        int outer_node_index = 0;
        const int MAX_ITERS = 5;
        for (int i = 0; i < MAX_ITERS; i++)
        {
            outer_node_index += static_cast<int>(std::floor(t));    // round towards negative infinity
            outer_node_index = std::clamp(outer_node_index, 0, outer_nodes.size()-2);   // and clamp
            
            t = Math::projectPointOntoLine(inner_node.position, outer_nodes[outer_node_index].position, outer_nodes[outer_node_index+1].position);

            if (t >= 0 && t <= 1)
                break;
        }

        // create a point-on-line constraint between the inner tube node and the outer tube segment
        _addConstraint<Constraint::PointLineConstraint>(&inner_node, &outer_nodes[outer_node_index], &outer_nodes[outer_node_index+1]);
    }
}

} // namespace SimObject