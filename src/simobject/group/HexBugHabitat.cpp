#include "simobject/group/HexBugHabitat.hpp"

#include "simobject/group/HexBug.hpp"

namespace SimObject
{

HexBugHabitat::HexBugHabitat(const Config::HexBugHabitatConfig& config)
    : XPBDObjectGroup_Base(config),
    _center(config.initialPosition()),
    _wall_length(config.wallLength()),
    _wall_thickness(config.wallThickness()),
    _wall_height(config.wallHeight()),
    _gap_size(config.gapSize()),
    _gaps(config.gaps()),
    _wall_color(config.wallColor())
{

}

void HexBugHabitat::setup()
{

    _objects.template reserve<XPBDRigidBox>(12);

    // create walls
    for (int i = 0; i < 6; i++)
    {
        Real apothem = _wall_length * std::sqrt(3.0)/2;
        Real extend = _wall_thickness/4.0 * std::sqrt(3.0);
        Real x = _center[0] + (apothem - extend) * std::cos(i * M_PI/3.0);
        Real z = _center[2] + (apothem - extend) * std::sin(i * M_PI/3.0);

        if (_gaps[i])
        {
            Real x1 = x - (_wall_length + _gap_size)/4.0*std::sin(i * M_PI/3.0);
            Real z1 = z + (_wall_length + _gap_size)/4.0*std::cos(i * M_PI/3.0);
            Config::XPBDRigidBoxConfig wall_config1(
                "wall_left"+std::to_string(i), Vec3r(x1, _wall_height/2, z1), Vec3r(0,-i*60,0), Vec3r::Zero(), Vec3r::Zero(), true, 0.1, 0.0,
                100, true, Vec3r(_wall_thickness, _wall_height, (_wall_length - _gap_size)/2)
            );
            wall_config1.renderConfig().setColor(_wall_color);
            _objects.template emplace_back<XPBDRigidBox>(wall_config1);

            Real x2 = x - (_wall_length + _gap_size)/4.0*-std::sin(i * M_PI/3.0);
            Real z2 = z + (_wall_length + _gap_size)/4.0*-std::cos(i * M_PI/3.0);
            Config::XPBDRigidBoxConfig wall_config2(
                "wall_left"+std::to_string(i), Vec3r(x2, _wall_height/2, z2), Vec3r(0,-i*60,0), Vec3r::Zero(), Vec3r::Zero(), true, 0.1, 0.0,
                100, true, Vec3r(_wall_thickness, _wall_height, (_wall_length - _gap_size)/2)
            );
            wall_config2.renderConfig().setColor(_wall_color);
            _objects.template emplace_back<XPBDRigidBox>(wall_config2);
        }
        else
        {
            Config::XPBDRigidBoxConfig wall_config(
                "wall" + std::to_string(i), Vec3r(x, _wall_height/2, z), Vec3r(0,-i*60,0), Vec3r::Zero(), Vec3r::Zero(), true, 0.1, 0.0,
                100, true, Vec3r(_wall_thickness, _wall_height, _wall_length)
            );
            wall_config.renderConfig().setColor(_wall_color);
            _objects.template emplace_back<XPBDRigidBox>(wall_config);
        }
        
        
    }
    

    
}

} // namespace SimObject