#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidBoxConfig.hpp"

namespace SimObject
{

class XPBDRigidBox : public XPBDRigidBody_Base
{
public:
    XPBDRigidBox(const Config::XPBDRigidBoxConfig& config);

    virtual ~XPBDRigidBox();

    // Move operations
    XPBDRigidBox(XPBDRigidBox&&) noexcept;
    XPBDRigidBox& operator=(XPBDRigidBox&&) noexcept;
    
    // Delete copy operations
    XPBDRigidBox(const XPBDRigidBox&) = delete;
    XPBDRigidBox& operator=(const XPBDRigidBox&) = delete;

    Vec3r size() const { return _size; }

private:
    Vec3r _size;
};

} // namespace SimObject