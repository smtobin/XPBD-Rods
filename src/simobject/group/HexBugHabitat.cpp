#include "simobject/group/HexBugHabitat.hpp"

#include "simobject/group/HexBug.hpp"

namespace SimObject
{

HexBugHabitat::HexBugHabitat(const Config::HexBugHabitatConfig& config)
    : XPBDObjectGroup_Base(config),
    _wall_size(config.wallSize()),
    _wall_color(config.wallColor())
{

}

void HexBugHabitat::setup()
{

    _objects.template reserve<XPBDRigidBox>(6);

    // create walls
    for (int i = 0; i < 6; i++)
    {
        Real wall_length = _wall_size[2];
        Real x = wall_length * std::cos(i * M_PI/3.0);
        Real z = wall_length * std::sin(i * M_PI/3.0);
        Config::XPBDRigidBoxConfig wall_config(
            "wall" + std::to_string(i), Vec3r(x, _wall_size[1]/2, z), Vec3r(0,-i*60,0), Vec3r::Zero(), Vec3r::Zero(), true,
            100, true, _wall_size
        );
        _objects.template emplace_back<XPBDRigidBox>(wall_config);
    }
    

    
}

} // namespace SimObject