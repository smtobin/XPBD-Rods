#pragma once

#include "common/common.hpp"
#include "simobject/OrientedParticle.hpp"

namespace Constraint
{

template <int ConstraintDim_, int NumParticles_>
class XPBDConstraint
{
    public:
    constexpr static int ConstraintDim = ConstraintDim_;
    constexpr static int NumParticles = NumParticles_;
    constexpr static int StateDim = NumParticles_ * SimObject::OrientedParticle::DOF;

    using ConstraintVecType = Eigen::Vector<Real, ConstraintDim_>;
    using AlphaVecType = Eigen::Vector<Real, ConstraintDim_>;
    using GradientMatType = Eigen::Matrix<Real, ConstraintDim_, StateDim>;
    using SingleParticleGradientMatType = Eigen::Matrix<Real, ConstraintDim_, SimObject::OrientedParticle::DOF>; 
    using ParticlePtrArray = std::array<SimObject::OrientedParticle*, NumParticles>;
    using CachedSingleParticleGradientArray = std::array<SingleParticleGradientMatType, NumParticles>;

    public:
    XPBDConstraint(const ParticlePtrArray& particles, const AlphaVecType& alpha)
        : _particles(particles), _alpha(alpha)
    {
    }

    XPBDConstraint(std::initializer_list<SimObject::OrientedParticle*> particles_list, const AlphaVecType& alpha)
        : _particles{}, _alpha(alpha)
    {
        std::copy(particles_list.begin(), particles_list.end(), _particles.begin());
    }

    virtual ~XPBDConstraint() = default;

    virtual ConstraintVecType evaluate() const = 0;
    virtual GradientMatType gradient(bool update_cache=true) const = 0;

    virtual SingleParticleGradientMatType singleParticleGradient(const SimObject::OrientedParticle* particle_ptr, bool use_cache=false) const = 0;

    const AlphaVecType& alpha() const { return _alpha; }
    const ParticlePtrArray& particles() const { return _particles; }

    protected:
    ParticlePtrArray _particles;
    AlphaVecType _alpha;
    mutable CachedSingleParticleGradientArray _cached_gradients;
};

} // namespace Constraint