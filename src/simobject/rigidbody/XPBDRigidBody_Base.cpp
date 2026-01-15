#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"

#include "common/constraint_containers.hpp"

namespace SimObject
{
XPBDRigidBody_Base::XPBDRigidBody_Base(const Config::XPBDRigidBodyConfig& config)
    : XPBDObject_Base(config), _com()
{
    _com.position = config.initialPosition();
    _com.orientation = Math::RotMatFromXYZEulerAngles(config.initialRotation());
    _com.lin_velocity = config.initialVelocity();
    _com.ang_velocity = config.initialAngularVelocity();

    _prev_position = _com.position;
    _prev_orientation = _com.orientation;

    // Derived classes are responsible for setting the mass and rotational inertia
    _com.mass = -1;
    _com.Ib = Vec3r::Zero();
}

XPBDRigidBody_Base::~XPBDRigidBody_Base() = default;
XPBDRigidBody_Base::XPBDRigidBody_Base(XPBDRigidBody_Base&&) noexcept = default;
XPBDRigidBody_Base& XPBDRigidBody_Base::operator=(XPBDRigidBody_Base&&) noexcept = default;


} // namespace SimObject