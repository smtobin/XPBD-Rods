#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

#include "common/common.hpp"

namespace Constraint
{

template <int ConstraintDim_, int StateDim_>
class XPBDConstraint
{
    public:
    constexpr static int ConstraintDim = ConstraintDim_;
    constexpr static int StateDim = StateDim_;

    using ConstraintVecType = Eigen::Vector<Real, ConstraintDim_>;
    using AlphaVecType = Eigen::Vector<Real, ConstraintDim_>;
    using GradientMatType = Eigen::Matrix<Real, ConstraintDim_, StateDim_>;

    public:
    XPBDConstraint(const AlphaVecType& alpha)
        : _alpha(alpha)
    {
    }

    virtual ConstraintVecType evaluate() const = 0;
    virtual GradientMatType gradient() const = 0;

    const AlphaVecType& alpha() const { return _alpha; }

    protected:
    AlphaVecType _alpha;
};

} // namespace Constraint

#endif // __CONSTRAINT_HPP