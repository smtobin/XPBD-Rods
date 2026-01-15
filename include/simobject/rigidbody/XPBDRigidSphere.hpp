#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidSphereConfig.hpp"

namespace SimObject
{

class XPBDRigidSphere : public XPBDRigidBody_Base
{
public:
    XPBDRigidSphere(const Config::XPBDRigidSphereConfig& config);

    virtual ~XPBDRigidSphere();

    // Move operations
    XPBDRigidSphere(XPBDRigidSphere&&) noexcept;
    XPBDRigidSphere& operator=(XPBDRigidSphere&&) noexcept;
    
    // Delete copy operations
    XPBDRigidSphere(const XPBDRigidSphere&) = delete;
    XPBDRigidSphere& operator=(const XPBDRigidSphere&) = delete;

    Real radius() const { return _radius; }

private:
    Real _radius;
};

} // namespace SimObject