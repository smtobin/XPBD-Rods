#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

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
    using NodeIndexArray = std::array<int, NumParticles>;
    using CachedSingleParticleGradientArray = std::array<SingleParticleGradientMatType, NumParticles>;

    public:
    XPBDConstraint(const AlphaVecType& alpha)
        : _alpha(alpha)
    {
    }

    virtual ConstraintVecType evaluate() const = 0;
    virtual GradientMatType gradient(bool update_cache=true) const = 0;

    virtual SingleParticleGradientMatType singleNodeGradient(int node_index, bool use_cache=false) const = 0;

    const AlphaVecType& alpha() const { return _alpha; }
    const NodeIndexArray& nodeIndices() const { return _node_indices; }

    protected:
    AlphaVecType _alpha;
    NodeIndexArray _node_indices;
    mutable CachedSingleParticleGradientArray _cached_gradients;
};

} // namespace Constraint

#endif // __CONSTRAINT_HPP