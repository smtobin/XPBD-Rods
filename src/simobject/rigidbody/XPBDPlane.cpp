#include "simobject/rigidbody/XPBDPlane.hpp"

namespace SimObject
{

XPBDPlane::XPBDPlane(const Config::XPBDPlaneConfig& plane_config)
    : XPBDRigidBody_Base(plane_config), 
    _width(plane_config.width()), _height(plane_config.height()), _normal(plane_config.normal())
{
    _com.mass = _width * _height * plane_config.density();  // density is assumed to be per unit area
    _com.Ib = _com.mass * Vec3r(_width * _width/12, _height * _height/12, (_width * _width + _height * _height)/12);

    _com.fixed = true;
    _com.orientation = Mat3r::Identity();
}

AABB XPBDPlane::boundingBox() const
{
    Mat3r R_abs = _com.orientation.cwiseAbs();
    Vec3r AABB_size = R_abs.col(0) * _width + R_abs.col(1) * _height;
    AABB bbox;
    bbox.min = _com.position - AABB_size/2;
    bbox.max = _com.position + AABB_size/2;

    return bbox;
}

} // namespace SimObject