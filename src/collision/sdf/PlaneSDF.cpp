#include "collision/sdf/PlaneSDF.hpp"

#include "simobject/rigidbody/XPBDPlane.hpp"

namespace Collision
{

PlaneSDF::PlaneSDF(const SimObject::XPBDPlane* plane)
    : SDF(), _plane(plane)
{}

inline Real PlaneSDF::evaluate(const Vec3r& x) const
{
    // the distance below the plane
    return (x - _plane->com().position).dot(_plane->normal());
}

inline Vec3r PlaneSDF::gradient(const Vec3r& /* x */) const
{
    // the gradient is always the plane normal
    return _plane->normal();
}

} // namespace Geometry