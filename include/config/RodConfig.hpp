#ifndef __ROD_CONFIG_HPP
#define __ROD_CONFIG_HPP

#include "config/Config.hpp"
#include "config/ObjectRenderConfig.hpp"

namespace Config
{

class RodConfig : public Config
{
    public:
    explicit RodConfig() 
        : Config(), _render_config()
    {

    }

    explicit RodConfig(const YAML::Node& node)
        : Config(node), _render_config(node)
    {
        _extractParameter("initial-base-position", node, _initial_base_position);
        _extractParameter("initial-base-rotation", node, _initial_base_rotation);
        _extractParameter("initial-velocity", node, _initial_velocity);
        _extractParameter("initial-angular-velocity", node, _initial_angular_velocity);

        _extractParameter("base-fixed", node, _base_fixed);
        _extractParameter("tip-fixed", node, _tip_fixed);

        _extractParameter("length", node, _length);
        _extractParameter("diameter", node, _diameter);
        _extractParameter("nodes", node, _nodes);

        _extractParameter("density", node, _density);
        _extractParameter("E", node, _E);
        _extractParameter("nu", node, _nu);
        
    }

    explicit RodConfig(const std::string& name, const Vec3r& initial_base_position, const Vec3r& initial_base_rotation,
                          const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity,
                        bool base_fixed, bool tip_fixed,
                        Real length, Real diameter, int nodes,
                        Real density, Real E, Real nu)
        : Config(name), _render_config()
    {
        _initial_base_position.value = initial_base_position;
        _initial_base_rotation.value = initial_base_rotation;
        _initial_velocity.value = initial_velocity;
        _initial_angular_velocity.value = initial_angular_velocity;

        _base_fixed.value = base_fixed;
        _tip_fixed.value = tip_fixed;

        _length.value = length;
        _diameter.value = diameter;
        _nodes.value = nodes;
        
        _density.value = density;
        _E.value = E;
        _nu.value = nu;
    }

    const Vec3r& initialBasePosition() const { return _initial_base_position.value; }
    const Vec3r& initialBaseRotation() const { return _initial_base_rotation.value; }
    const Vec3r& initialVelocity() const { return _initial_velocity.value; }
    const Vec3r& initialAngularVelocity() const { return _initial_angular_velocity.value; }

    bool baseFixed() const { return _base_fixed.value; }
    bool tipFixed() const { return _tip_fixed.value; }

    Real length() const { return _length.value; }
    Real diameter() const { return _diameter.value; }
    int nodes() const { return _nodes.value; }

    Real density() const { return _density.value; }
    Real E() const { return _E.value; }
    Real nu() const { return _nu.value; }

    const ObjectRenderConfig& renderConfig() const { return _render_config; }

    protected:
    ConfigParameter<Vec3r> _initial_base_position = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));
    ConfigParameter<Vec3r> _initial_base_rotation = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));
    ConfigParameter<Vec3r> _initial_velocity = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));
    ConfigParameter<Vec3r> _initial_angular_velocity = ConfigParameter<Vec3r>(Vec3r(0.0, 0.0, 0.0));

    ConfigParameter<bool> _base_fixed = ConfigParameter<bool>(true);
    ConfigParameter<bool> _tip_fixed = ConfigParameter<bool>(false);

    ConfigParameter<Real> _length = ConfigParameter<Real>(1.0);
    ConfigParameter<Real> _diameter = ConfigParameter<Real>(0.1);
    ConfigParameter<int> _nodes = ConfigParameter<int>(20);
    
    ConfigParameter<Real> _density = ConfigParameter<Real>(1000);
    ConfigParameter<Real> _E = ConfigParameter<Real>(3e6);
    ConfigParameter<Real> _nu = ConfigParameter<Real>(0.45);

    ObjectRenderConfig _render_config;
};

} // namespace Config

#endif // __ROD_CONFIG_HPP