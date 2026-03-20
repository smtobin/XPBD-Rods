#pragma once

#include "common/common.hpp"
#include "simobject/OrientedParticle.hpp"
#include "simobject/Particle.hpp"

namespace Constraint
{

template <int ConstraintDim_, int NumOrientedParticles_, int NumParticles_>
class XPBDConstraint
{
    public:
    constexpr static int ConstraintDim = ConstraintDim_;
    constexpr static int NumOrientedParticles = NumOrientedParticles_;
    constexpr static int NumParticles = NumParticles_;
    constexpr static int StateDim = NumOrientedParticles_ * SimObject::OrientedParticle::DOF + NumParticles_ * SimObject::Particle::DOF;

    using ConstraintVecType = Eigen::Vector<Real, ConstraintDim_>;
    using AlphaVecType = Eigen::Vector<Real, ConstraintDim_>;
    using GradientMatType = Eigen::Matrix<Real, ConstraintDim_, StateDim>;
    using OrientedParticlePtrArray = std::array<SimObject::OrientedParticle*, NumOrientedParticles>;
    using ParticlePtrArray = std::array<SimObject::Particle*, NumParticles>;

    public:
    XPBDConstraint(const OrientedParticlePtrArray& oriented_particles, const AlphaVecType& alpha)
        : _oriented_particles(oriented_particles), _alpha(alpha)
    {
    }

    XPBDConstraint(std::initializer_list<SimObject::OrientedParticle*> oriented_particles_list, const AlphaVecType& alpha)
        : _oriented_particles{}, _alpha(alpha)
    {
        std::copy(oriented_particles_list.begin(), oriented_particles_list.end(), _oriented_particles.begin());
    }

    virtual ~XPBDConstraint() = default;

    virtual bool isInequality() const { return false; }

    virtual ConstraintVecType evaluate() const = 0;
    virtual GradientMatType gradient() const = 0;

    const AlphaVecType& alpha() const { return _alpha; }
    const OrientedParticlePtrArray& particles() const { return _oriented_particles; }

    protected:
    OrientedParticlePtrArray _oriented_particles;
    AlphaVecType _alpha;
};

} // namespace Constraint