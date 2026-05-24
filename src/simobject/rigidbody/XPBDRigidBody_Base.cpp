#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"

namespace SimObject
{
XPBDRigidBody_Base::XPBDRigidBody_Base(const Config::XPBDRigidBodyConfig& config)
    : XPBDObject_Base(config), _com()
{
    _com.position = config.initialPosition();
    _com.orientation = Math::RotMatFromXYZEulerAngles(config.initialRotation());
    _com.lin_velocity = config.initialVelocity();
    _com.ang_velocity = config.initialAngularVelocity();

    _com.fixed = config.fixed();

    _com.prev_position = _com.position;
    _com.prev_orientation = _com.orientation;
    _com.prev_lin_velocity = _com.lin_velocity;
    _com.prev_ang_velocity = _com.ang_velocity;

    // Derived classes are responsible for setting the mass and rotational inertia
    _com.mass = -1;
    _com.Ib = Vec3r::Zero();
}


} // namespace SimObject