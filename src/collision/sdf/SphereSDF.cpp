#include "collision/sdf/SphereSDF.hpp"

#include "simobject/rigidbody/XPBDRigidSphere.hpp"

namespace Collision
{

SphereSDF::SphereSDF(const SimObject::XPBDRigidSphere* sphere)
    : SDF(), _sphere(sphere)
{}

inline Real SphereSDF::evaluate(const Vec3r& x) const
{
    // the distance from any point the surface of the sphere is simply the distance of the point to the sphere center minus the radius
    return (x - _sphere->com().position).norm() - _sphere->radius();
}

inline Vec3r SphereSDF::gradient(const Vec3r& x) const
{
    // the gradient simply is a normalized vector pointing out from the sphere center in the direction of x
    return (x - _sphere->com().position).normalized();
}

} // namespace Geometry