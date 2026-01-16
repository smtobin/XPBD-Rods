#include "simobject/rigidbody/XPBDRigidSphere.hpp"

namespace SimObject
{

XPBDRigidSphere::XPBDRigidSphere(const Config::XPBDRigidSphereConfig& config)
    : XPBDRigidBody_Base(config), _radius(config.radius())
{
    // mass = 4/3 * pi * r^3 * density
    _com.mass = config.density() * 4.0/3.0 * M_PI * _radius * _radius * _radius;
    // moment of inertia = 2/5 * m * r^2
    _com.Ib = 0.4 * _com.mass * _radius * _radius * Vec3r::Ones();
}

XPBDRigidSphere::~XPBDRigidSphere() = default;
XPBDRigidSphere::XPBDRigidSphere(XPBDRigidSphere&&) noexcept = default;
XPBDRigidSphere& XPBDRigidSphere::operator=(XPBDRigidSphere&&) noexcept = default;

} // namespace SimObject