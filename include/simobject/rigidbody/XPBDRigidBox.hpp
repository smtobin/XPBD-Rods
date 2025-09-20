#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidBoxConfig.hpp"

namespace SimObject
{

class XPBDRigidBox : public XPBDRigidBody_Base
{
public:
    XPBDRigidBox(const Config::XPBDRigidBoxConfig& config)
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

    Vec3r size() const { return _size; }

private:
    Vec3r _size;
};

} // namespace SimObject