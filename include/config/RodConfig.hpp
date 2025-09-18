#ifndef __ROD_CONFIG_HPP
#define __ROD_CONFIG_HPP

#include "config/XPBDObjectConfig.hpp"
#include "config/ObjectRenderConfig.hpp"

namespace Config
{

class RodConfig : public XPBDObjectConfig
{
    public:
    explicit RodConfig() 
        : XPBDObjectConfig(), _render_config()
    {

    }

    explicit RodConfig(const YAML::Node& node)
        : XPBDObjectConfig(node), _render_config(node)
    {
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
        : XPBDObjectConfig(name, initial_base_position, initial_base_rotation, initial_velocity, initial_angular_velocity),
         _render_config()
    {
        _base_fixed.value = base_fixed;
        _tip_fixed.value = tip_fixed;

        _length.value = length;
        _diameter.value = diameter;
        _nodes.value = nodes;
        
        _density.value = density;
        _E.value = E;
        _nu.value = nu;
    }

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