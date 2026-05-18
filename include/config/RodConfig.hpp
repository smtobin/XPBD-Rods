#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

enum class RodElementType
{
    RIGID_BODY,
    LINEAR,
    QUADRATIC,
    CUBIC,
    CUBIC_HERMITE
};
static std::map<std::string, RodElementType> ROD_ELEMENT_TYPE_OPTIONS()
{
    static std::map<std::string, RodElementType> rod_element_options{{"Rigid Body", RodElementType::RIGID_BODY},
                                                                    {"Linear", RodElementType::LINEAR},
                                                                    {"Quadratic", RodElementType::QUADRATIC},
                                                                    {"Cubic", RodElementType::CUBIC},
                                                                    {"Cubic Hermite", RodElementType::CUBIC_HERMITE}};
    return rod_element_options;
}

class RodConfig : public XPBDObjectConfig
{
    public:
    explicit RodConfig() 
        : XPBDObjectConfig()
    {

    }

    explicit RodConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
        _extractParameterWithOptions("element-type", node, _element_type, ROD_ELEMENT_TYPE_OPTIONS());
        _extractParameter("base-fixed", node, _base_fixed);
        _extractParameter("tip-fixed", node, _tip_fixed);

        _extractParameter("global-solve", node, _global_solve);

        _extractParameter("length", node, _length);
        _extractParameter("diameter", node, _diameter);
        _extractParameter("elements", node, _elements);

        _extractParameter("density", node, _density);
        _extractParameter("E", node, _E);
        _extractParameter("nu", node, _nu);

        _extractParameter("curvature", node, _curvature);

        _extractParameter("tip-force", node, _tip_force);
        
    }

    explicit RodConfig(const std::string& name, const Vec3r& initial_base_position, const Vec3r& initial_base_rotation,
                          const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, bool collisions,
                        RodElementType element_type,
                        bool base_fixed, bool tip_fixed, bool global_solve,
                        Real length, Real diameter, int elements,
                        Real density, Real E, Real nu, Vec3r curvature)
        : XPBDObjectConfig(name, initial_base_position, initial_base_rotation, initial_velocity, initial_angular_velocity, collisions)
    {
        _element_type.value = element_type;

        _base_fixed.value = base_fixed;
        _tip_fixed.value = tip_fixed;

        _global_solve.value = global_solve;

        _length.value = length;
        _diameter.value = diameter;
        _elements.value = elements;
        
        _density.value = density;
        _E.value = E;
        _nu.value = nu;

        _curvature.value = curvature;
    }

    RodElementType elementType() const { return _element_type.value; }

    bool baseFixed() const { return _base_fixed.value; }
    bool tipFixed() const { return _tip_fixed.value; }

    bool globalSolve() const { return _global_solve.value; }

    Real length() const { return _length.value; }
    Real diameter() const { return _diameter.value; }
    int elements() const { return _elements.value; }

    Real density() const { return _density.value; }
    Real E() const { return _E.value; }
    Real nu() const { return _nu.value; }

    Vec3r curvature() const { return _curvature.value; }

    Vec3r tipForce() const { return _tip_force.value; }

    protected:
    ConfigParameter<RodElementType> _element_type = ConfigParameter<RodElementType>(RodElementType::LINEAR);

    ConfigParameter<bool> _base_fixed = ConfigParameter<bool>(true);
    ConfigParameter<bool> _tip_fixed = ConfigParameter<bool>(false);

    ConfigParameter<bool> _global_solve = ConfigParameter<bool>(true);

    ConfigParameter<Real> _length = ConfigParameter<Real>(1.0);
    ConfigParameter<Real> _diameter = ConfigParameter<Real>(0.1);
    ConfigParameter<int> _elements = ConfigParameter<int>(20);
    
    ConfigParameter<Real> _density = ConfigParameter<Real>(1000);
    ConfigParameter<Real> _E = ConfigParameter<Real>(3e6);
    ConfigParameter<Real> _nu = ConfigParameter<Real>(0.45);

    ConfigParameter<Vec3r> _curvature = ConfigParameter<Vec3r>(Vec3r(0,0,0));

    ConfigParameter<Vec3r> _tip_force = ConfigParameter<Vec3r>(Vec3r(0,0,0));
};

} // namespace Config