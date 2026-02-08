#include "simobject/rigidbody/XPBDPlane.hpp"

namespace SimObject
{

XPBDPlane::XPBDPlane(const Config::XPBDPlaneConfig& plane_config)
    : XPBDRigidBody_Base(plane_config), 
    _width(plane_config.width()), _height(plane_config.height()), _normal(plane_config.normal().normalized())
{
    _com.mass = _width * _height * plane_config.density();  // density is assumed to be per unit area
    _com.Ib = _com.mass * Vec3r(_width * _width/12, _height * _height/12, (_width * _width + _height * _height)/12);

    _com.fixed = true;
    
    // calculate the orientation based on the desired normal
    // by default, plane is created in XY plane, with Z-axis as the normal
    Vec3r default_normal(0, 0, 1);

    Vec3r axis = default_normal.cross(_normal);
    double axis_length = axis.norm();

    if (axis_length > 1e-6)
    {
        axis /= axis_length;
        Real angle = std::acos(default_normal.dot(_normal));
        Vec3r exp_coords = angle * axis;
        
        _com.orientation = Math::Exp_so3(exp_coords);
    }
    else if (_normal.dot(default_normal) < 0)
    {
        _com.orientation = Math::Exp_so3(Vec3r(M_PI, 0, 0));
    }
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