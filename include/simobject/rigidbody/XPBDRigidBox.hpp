#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidBoxConfig.hpp"

namespace SimObject
{

class XPBDRigidBox : public XPBDRigidBody_Base
{
public:
    XPBDRigidBox(const Config::XPBDRigidBoxConfig& config);

    Vec3r size() const { return _size; }

    virtual AABB boundingBox() const override;

private:
    Vec3r _size;
};

} // namespace SimObject