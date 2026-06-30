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

    XPBDConstraint(const OrientedParticlePtrArray& oriented_particles, const ParticlePtrArray& particles, const AlphaVecType& alpha, Real beta=0)
        : _oriented_particles(oriented_particles), _particles(particles), _alpha(alpha), _beta(beta)
    {
    }

    XPBDConstraint(std::initializer_list<SimObject::OrientedParticle*> oriented_particles_list, std::initializer_list<SimObject::Particle*> particles_list, AlphaVecType& alpha, Real beta=0)
        : _oriented_particles{}, _particles{}, _alpha(alpha), _beta(beta)
    {
        std::copy(oriented_particles_list.begin(), oriented_particles_list.end(), _oriented_particles.begin());
        std::copy(particles_list.begin(), particles_list.end(), _particles.begin());
    }

    /** Enable reduced constructors for when the number of positional particles = 0 */

    template<int M = NumParticles_, typename std::enable_if<M == 0, int>::type = 0>
    XPBDConstraint(const OrientedParticlePtrArray& oriented_particles, const AlphaVecType& alpha, Real beta=0)
        : _oriented_particles(oriented_particles), _alpha(alpha), _beta(beta)
    {
    }

    template<int M = NumParticles_, typename std::enable_if<M == 0, int>::type = 0>
    XPBDConstraint(std::initializer_list<SimObject::OrientedParticle*> oriented_particles_list, const AlphaVecType& alpha, Real beta=0)
        : _oriented_particles{}, _particles{}, _alpha(alpha), _beta(beta)
    {
        std::copy(oriented_particles_list.begin(), oriented_particles_list.end(), _oriented_particles.begin());
    }


    /** Enable reduced constructors for when the number of oriented particles = 0 */
    
    template<int M = NumOrientedParticles_, typename std::enable_if<M == 0, int>::type = 0>
    XPBDConstraint(const ParticlePtrArray& particles, const AlphaVecType& alpha, Real beta=0)
        : _particles(particles), _alpha(alpha), _beta(beta)
    {
    }

    template<int M = NumOrientedParticles_, typename std::enable_if<M == 0, int>::type = 0>
    XPBDConstraint(std::initializer_list<SimObject::Particle*> particles_list, const AlphaVecType& alpha, Real beta=0)
        : _oriented_particles{}, _particles{}, _alpha(alpha), _beta(beta)
    {
        std::copy(particles_list.begin(), particles_list.end(), _particles.begin());
    }

    virtual ~XPBDConstraint() = default;

    virtual bool isInequality() const { return false; }

    virtual ConstraintVecType evaluate() const = 0;
    virtual GradientMatType gradient() const = 0;

    const AlphaVecType& alpha() const { return _alpha; }
    Real beta() const { return _beta; }
    const OrientedParticlePtrArray& orientedParticles() const { return _oriented_particles; }
    const ParticlePtrArray& particles() const { return _particles; }

    protected:
    OrientedParticlePtrArray _oriented_particles;
    ParticlePtrArray _particles;
    AlphaVecType _alpha;

    /** Rayleigh dissipation coefficient.
     * Scaled by inverse stiffness (alpha) so that the damping ratio is consistent across the individual constraints.
     */
    Real _beta;
};

} // namespace Constraint