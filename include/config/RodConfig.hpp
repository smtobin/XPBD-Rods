#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

enum class RodElementType
{
    NONE,
    LINEAR,
    QUADRATIC
};
static std::map<std::string, RodElementType> ROD_ELEMENT_TYPE_OPTIONS()
{
    static std::map<std::string, RodElementType> rod_element_options{{"None", RodElementType::NONE},
                                                                    {"Linear", RodElementType::LINEAR},
                                                                    {"Quadratic", RodElementType::QUADRATIC}};
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

        _extractParameter("length", node, _length);
        _extractParameter("diameter", node, _diameter);
        _extractParameter("elements", node, _elements);

        _extractParameter("density", node, _density);
        _extractParameter("E", node, _E);
        _extractParameter("nu", node, _nu);
        
    }

    explicit RodConfig(const std::string& name, const Vec3r& initial_base_position, const Vec3r& initial_base_rotation,
                          const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity,\
                        RodElementType element_type,
                        bool base_fixed, bool tip_fixed,
                        Real length, Real diameter, int elements,
                        Real density, Real E, Real nu)
        : XPBDObjectConfig(name, initial_base_position, initial_base_rotation, initial_velocity, initial_angular_velocity)
    {
        _element_type.value = element_type;

        _base_fixed.value = base_fixed;
        _tip_fixed.value = tip_fixed;

        _length.value = length;
        _diameter.value = diameter;
        _elements.value = elements;
        
        _density.value = density;
        _E.value = E;
        _nu.value = nu;
    }

    RodElementType elementType() const { return _element_type.value; }

    bool baseFixed() const { return _base_fixed.value; }
    bool tipFixed() const { return _tip_fixed.value; }

    Real length() const { return _length.value; }
    Real diameter() const { return _diameter.value; }
    int elements() const { return _elements.value; }

    Real density() const { return _density.value; }
    Real E() const { return _E.value; }
    Real nu() const { return _nu.value; }

    protected:
    ConfigParameter<RodElementType> _element_type = ConfigParameter<RodElementType>(RodElementType::LINEAR);

    ConfigParameter<bool> _base_fixed = ConfigParameter<bool>(true);
    ConfigParameter<bool> _tip_fixed = ConfigParameter<bool>(false);

    ConfigParameter<Real> _length = ConfigParameter<Real>(1.0);
    ConfigParameter<Real> _diameter = ConfigParameter<Real>(0.1);
    ConfigParameter<int> _elements = ConfigParameter<int>(20);
    
    ConfigParameter<Real> _density = ConfigParameter<Real>(1000);
    ConfigParameter<Real> _E = ConfigParameter<Real>(3e6);
    ConfigParameter<Real> _nu = ConfigParameter<Real>(0.45);
};

} // namespace Config