#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

#include "common/common.hpp"
#include "rod/XPBDRodNode.hpp"

namespace Constraint
{

template <int ConstraintDim_, int NumNodes_>
class XPBDConstraint
{
    public:
    constexpr static int ConstraintDim = ConstraintDim_;
    constexpr static int NumNodes = NumNodes_;
    constexpr static int StateDim = NumNodes_ * Rod::XPBDRodNode::NODE_DOF;

    using ConstraintVecType = Eigen::Vector<Real, ConstraintDim_>;
    using AlphaVecType = Eigen::Vector<Real, ConstraintDim_>;
    using GradientMatType = Eigen::Matrix<Real, ConstraintDim_, StateDim>;
    using SingleNodeGradientMatType = Eigen::Matrix<Real, ConstraintDim_, Rod::XPBDRodNode::NODE_DOF>; 
    using NodeIndexArray = std::array<int, NumNodes>;

    public:
    XPBDConstraint(const AlphaVecType& alpha)
        : _alpha(alpha)
    {
    }

    virtual ConstraintVecType evaluate() const = 0;
    virtual GradientMatType gradient() const = 0;

    virtual SingleNodeGradientMatType singleNodeGradient(int node_index) const = 0;

    const AlphaVecType& alpha() const { return _alpha; }
    const NodeIndexArray& nodeIndices() const { return _node_indices; }

    protected:
    AlphaVecType _alpha;
    NodeIndexArray _node_indices;
};

} // namespace Constraint

#endif // __CONSTRAINT_HPP