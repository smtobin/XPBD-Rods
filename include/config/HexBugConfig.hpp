#pragma once

#include "config/XPBDObjectConfig.hpp"

namespace Config
{

class HexBugConfig : public XPBDObjectConfig
{
public:
    using SimObjectType = SimObject::HexBug;

    explicit HexBugConfig()
        : XPBDObjectConfig()
    {}

    explicit HexBugConfig(const YAML::Node& node)
        : XPBDObjectConfig(node)
    {
        _extractParameter("body-initial-position", node, _body_initial_position);
        _extractParameter("body-size", node, _body_size);
        _extractParameter("body-density", node, _body_density);

        _extractParameter("motor-angular-velocity", node, _motor_angular_velocity);

        _extractParameter("leg-stiffness", node, _leg_stiffness);
        _extractParameter("leg-length", node, _leg_length);
        _extractParameter("leg-length-increment", node, _leg_length_increment);
        _extractParameter("leg-diameter", node, _leg_diameter);
        _extractParameter("leg-curvature", node, _leg_curvature);
    }

    Vec3r bodyInitialPosition() const { return _body_initial_position.value; }
    Vec3r bodySize() const { return _body_size.value; }
    Real bodyDensity() const { return _body_density.value; }

    Real motorAngularVelocity() const { return _motor_angular_velocity.value; }

    Real legStiffness() const { return _leg_stiffness.value; }
    Real legLength() const { return _leg_length.value; }
    Real legLengthIncrement() const { return _leg_length_increment.value; }
    Real legDiameter() const { return _leg_diameter.value; }
    Vec3r legCurvature() const { return _leg_curvature.value; }
    
private:
    ConfigParameter<Vec3r> _body_initial_position = ConfigParameter<Vec3r>(Vec3r(0,0.023,0));
    ConfigParameter<Vec3r> _body_size = ConfigParameter<Vec3r>(Vec3r(0.0125, 0.007, 0.04));
    ConfigParameter<Real> _body_density = ConfigParameter<Real>(1030);

    ConfigParameter<Real> _motor_angular_velocity = ConfigParameter<Real>(220);

    ConfigParameter<Real> _leg_stiffness = ConfigParameter<Real>(1e6);
    ConfigParameter<Real> _leg_length = ConfigParameter<Real>(1.875e-2);
    ConfigParameter<Real> _leg_length_increment = ConfigParameter<Real>(0.5e-3);
    ConfigParameter<Real> _leg_diameter = ConfigParameter<Real>(2.5e-3);
    ConfigParameter<Vec3r> _leg_curvature = ConfigParameter<Vec3r>(Vec3r(10,0,0));

};

} // namespace Config