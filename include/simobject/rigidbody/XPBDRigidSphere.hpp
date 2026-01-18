#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidSphereConfig.hpp"

namespace SimObject
{

class XPBDRigidSphere : public XPBDRigidBody_Base
{
public:
    XPBDRigidSphere(const Config::XPBDRigidSphereConfig& config);

    Real radius() const { return _radius; }

private:
    Real _radius;
};

} // namespace SimObject