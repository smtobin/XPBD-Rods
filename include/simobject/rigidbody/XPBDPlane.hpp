#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDPlaneConfig.hpp"

namespace SimObject
{

class XPBDPlane : public XPBDRigidBody_Base
{

public:
    XPBDPlane(const Config::XPBDPlaneConfig& plane_config);

    Real width() const { return _width; }
    Real height() const { return _height; }
    const Vec3r& normal() const { return _normal; }

    virtual AABB boundingBox() const override;

private:
    Real _width;
    Real _height;
    Vec3r _normal;

};

} // namespace SimObject