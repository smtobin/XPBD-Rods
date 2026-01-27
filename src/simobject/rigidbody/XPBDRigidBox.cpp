#include "simobject/rigidbody/XPBDRigidBox.hpp"

namespace SimObject
{

XPBDRigidBox::XPBDRigidBox(const Config::XPBDRigidBoxConfig& config)
    : XPBDRigidBody_Base(config), _size(config.size())
{
    // mass = density * volume
    _com.mass = config.density() * _size[0] * _size[1] * _size[2];
    // moment of inertia = 1/12 * m * (d1^2 + d2^2)
    Real Ix = 1.0/12.0 * _com.mass * (_size[1] * _size[1] + _size[2] * _size[2]);
    Real Iy = 1.0/12.0 * _com.mass * (_size[0] * _size[0] + _size[2] * _size[2]);
    Real Iz = 1.0/12.0 * _com.mass * (_size[0] * _size[0] + _size[1] * _size[1]);
    _com.Ib = Vec3r(Ix, Iy, Iz);
}

AABB XPBDRigidBox::boundingBox() const
{
    Mat3r R_abs = _com.orientation.cwiseAbs();
    Vec3r AABB_size = R_abs.col(0) * _size[0] + R_abs.col(1) * _size[1] + R_abs.col(2) * _size[2];
    AABB bbox;
    bbox.min = _com.position - AABB_size/2;
    bbox.max = _com.position + AABB_size/2;

    return bbox;
}

}  // namespace SimObject