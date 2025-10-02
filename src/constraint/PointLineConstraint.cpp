#include "constraint/PointLineConstraint.hpp"

namespace Constraint
{

PointLineConstraint::PointLineConstraint(SimObject::OrientedParticle* point,
     SimObject::OrientedParticle* endpoint1,
      SimObject::OrientedParticle* endpoint2)
    : XPBDConstraint<1,3>({point, endpoint1, endpoint2}, AlphaVecType::Zero())
{

}

PointLineConstraint::ConstraintVecType PointLineConstraint::evaluate() const
{
    ConstraintVecType C;
    const Vec3r& p = _particles[0]->position;    // the point
    const Vec3r& x1 = _particles[1]->position;   // endpoint 1 of the line segment
    const Vec3r& x2 = _particles[2]->position;   // endpoint 2 of the line segment
    const Vec3r& cross_p = (p - x1).cross(p - x2);
    C[0] = cross_p.norm() / (x2 - x1).norm();

    return C;
}

PointLineConstraint::GradientMatType PointLineConstraint::gradient(bool update_cache) const
{
    GradientMatType grad = GradientMatType::Zero();

    const Vec3r& p = _particles[0]->position;    // the point
    const Vec3r& x1 = _particles[1]->position;   // endpoint 1 of the line segment
    const Vec3r& x2 = _particles[2]->position;   // endpoint 2 of the line segment
    const Vec3r& cross_p = (p - x1).cross(p - x2);
    const Real norm_cross_p = cross_p.norm();
    const Real norm_x1x2 = (x2 - x1).norm();

    const Vec3r dC_dp = cross_p.transpose() / norm_cross_p * Math::Skew3(x2 - x1) / norm_x1x2;
    const Vec3r dC_dx1 = norm_cross_p * (x2 - x1).transpose() / (norm_x1x2*norm_x1x2*norm_x1x2) + cross_p.transpose() / norm_cross_p / norm_x1x2 * Math::Skew3(p - x2);
    const Vec3r dC_dx2 = norm_cross_p * (x1 - x2).transpose() / (norm_x1x2*norm_x1x2*norm_x1x2) - cross_p.transpose() / norm_cross_p / norm_x1x2 * Math::Skew3(p - x1);


    grad.block<1,3>(0,0) = dC_dp;
    grad.block<1,3>(0,6) = dC_dx1;
    grad.block<1,3>(0,12) = dC_dx2;

    if (update_cache)
    {
        _cached_gradients[0].block<1,3>(0,0) = dC_dp;
        _cached_gradients[0].block<1,3>(0,3) = Vec3r::Zero();
        _cached_gradients[1].block<1,3>(0,0) = dC_dx1;
        _cached_gradients[1].block<1,3>(0,3) = Vec3r::Zero();
        _cached_gradients[2].block<1,3>(0,0) = dC_dx2;
        _cached_gradients[2].block<1,3>(0,3) = Vec3r::Zero();
    }

    return grad;
}

PointLineConstraint::SingleParticleGradientMatType PointLineConstraint::singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache) const
{
    
    if (particle_ptr == _particles[0])
    {
        if (use_cache)
            return _cached_gradients[0];

        const Vec3r& p = _particles[0]->position;    // the point
        const Vec3r& x1 = _particles[1]->position;   // endpoint 1 of the line segment
        const Vec3r& x2 = _particles[2]->position;   // endpoint 2 of the line segment
        const Vec3r& cross_p = (p - x1).cross(p - x2);
        const Real norm_cross_p = cross_p.norm();
        const Real norm_x1x2 = (x2 - x1).norm();
        const Vec3r dC_dp = cross_p.transpose() / norm_cross_p * Math::Skew3(x2 - x1) / norm_x1x2;

        SingleParticleGradientMatType grad = SingleParticleGradientMatType::Zero();
        grad.block<1,3>(0,0) = dC_dp;
        return grad;
    }
    if (particle_ptr == _particles[1])
    {
        if (use_cache)
            return _cached_gradients[1];
        
        const Vec3r& p = _particles[0]->position;    // the point
        const Vec3r& x1 = _particles[1]->position;   // endpoint 1 of the line segment
        const Vec3r& x2 = _particles[2]->position;   // endpoint 2 of the line segment
        const Vec3r& cross_p = (p - x1).cross(p - x2);
        const Real norm_cross_p = cross_p.norm();
        const Real norm_x1x2 = (x2 - x1).norm();
        const Vec3r dC_dx1 = norm_cross_p * (x2 - x1).transpose() / (norm_x1x2*norm_x1x2*norm_x1x2) + cross_p.transpose() / norm_cross_p / norm_x1x2 * Math::Skew3(p - x2);

        SingleParticleGradientMatType grad = SingleParticleGradientMatType::Zero();
        grad.block<1,3>(0,0) = dC_dx1;
        return grad;
    }
    if (particle_ptr == _particles[2])
    {
        if (use_cache)
            return _cached_gradients[2];
        
        const Vec3r& p = _particles[0]->position;    // the point
        const Vec3r& x1 = _particles[1]->position;   // endpoint 1 of the line segment
        const Vec3r& x2 = _particles[2]->position;   // endpoint 2 of the line segment
        const Vec3r& cross_p = (p - x1).cross(p - x2);
        const Real norm_cross_p = cross_p.norm();
        const Real norm_x1x2 = (x2 - x1).norm();
        const Vec3r dC_dx2 = norm_cross_p * (x1 - x2).transpose() / (norm_x1x2*norm_x1x2*norm_x1x2) - cross_p.transpose() / norm_cross_p / norm_x1x2 * Math::Skew3(p - x1);

        SingleParticleGradientMatType grad = SingleParticleGradientMatType::Zero();
        grad.block<1,3>(0,0) = dC_dx2;
        return grad;
    }
    else
    {
        return SingleParticleGradientMatType::Zero();
    }
}

} // namespace Constraint