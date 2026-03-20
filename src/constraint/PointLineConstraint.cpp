#include "constraint/PointLineConstraint.hpp"

#include <Eigen/Geometry>

namespace Constraint
{

PointLineConstraint::PointLineConstraint(SimObject::OrientedParticle* point,
     SimObject::OrientedParticle* endpoint1,
      SimObject::OrientedParticle* endpoint2)
    : XPBDConstraint<1,3,0>({point, endpoint1, endpoint2}, AlphaVecType::Zero())
{

}

PointLineConstraint::ConstraintVecType PointLineConstraint::evaluate() const
{
    ConstraintVecType C;
    const Vec3r& p = _oriented_particles[0]->position;    // the point
    const Vec3r& x1 = _oriented_particles[1]->position;   // endpoint 1 of the line segment
    const Vec3r& x2 = _oriented_particles[2]->position;   // endpoint 2 of the line segment
    const Vec3r& cross_p = (p - x1).cross(p - x2);
    C[0] = cross_p.norm() / (x2 - x1).norm();

    return C;
}

PointLineConstraint::GradientMatType PointLineConstraint::gradient() const
{
    GradientMatType grad = GradientMatType::Zero();

    const Vec3r& p = _oriented_particles[0]->position;    // the point
    const Vec3r& x1 = _oriented_particles[1]->position;   // endpoint 1 of the line segment
    const Vec3r& x2 = _oriented_particles[2]->position;   // endpoint 2 of the line segment
    const Vec3r& cross_p = (p - x1).cross(p - x2);
    const Real norm_cross_p = cross_p.norm();
    const Real norm_x1x2 = (x2 - x1).norm();

    const Real C = norm_cross_p / norm_x1x2;

    if (C >= 1e-10)
    {
        const Vec3r dC_dp = cross_p.transpose() / norm_cross_p * Math::Skew3(x2 - x1) / norm_x1x2;
        const Vec3r dC_dx1 = norm_cross_p * (x2 - x1).transpose() / (norm_x1x2*norm_x1x2*norm_x1x2) + cross_p.transpose() / norm_cross_p / norm_x1x2 * Math::Skew3(p - x2);
        const Vec3r dC_dx2 = norm_cross_p * (x1 - x2).transpose() / (norm_x1x2*norm_x1x2*norm_x1x2) - cross_p.transpose() / norm_cross_p / norm_x1x2 * Math::Skew3(p - x1);


        grad.block<1,3>(0,0) = dC_dp;
        grad.block<1,3>(0,6) = dC_dx1;
        grad.block<1,3>(0,12) = dC_dx2;
    }

    

    return grad;
}

} // namespace Constraint