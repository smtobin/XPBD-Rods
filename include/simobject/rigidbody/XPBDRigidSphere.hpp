#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidSphereConfig.hpp"

namespace SimObject
{

class XPBDRigidSphere : public XPBDRigidBody_Base
{
public:
    XPBDRigidSphere(const Config::XPBDRigidSphereConfig& config)
        : XPBDRigidBody_Base(config), _radius(config.radius())
    {
        // mass = 4/3 * pi * r^3 * density
        _com.mass = config.density() * 4.0/3.0 * M_PI * _radius * _radius * _radius;
        // moment of inertia = 2/5 * m * r^2
        _com.Ib = 0.4 * _com.mass * _radius * _radius * Vec3r::Ones();
    }

    Real radius() const { return _radius; }

private:
    Real _radius;
};

} // namespace SimObject