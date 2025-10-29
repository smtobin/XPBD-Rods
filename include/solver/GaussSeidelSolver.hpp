#pragma once

#include "common/common.hpp"
#include "common/VariadicVectorContainer.hpp"

#include "constraint/FixedJointConstraint.hpp"
#include "constraint/RodElasticConstraint.hpp"
#include "constraint/XPBDConstraintProjector.hpp"
#include "constraint/muller2020/MullerConstraintProjector.hpp"

#include <memory>
#include <vector>

namespace Solver
{

class GaussSeidelSolver
{
public:
    GaussSeidelSolver(Real dt, int num_iter);

    template <typename ConstraintType>
    void addConstraint(const ConstraintType* constraint)
    {
        /** TODO: Probably need constraint reference or something here. When std::vector reallocates, the pointer to the constraint used by the
         * constraint projector will become invalid, leading to segfault.
         */
        _constraints.template push_back<const ConstraintType*>(constraint);

        std::unique_ptr<Constraint::XPBDConstraintProjector_Base> new_projector =
             std::move( std::make_unique<Constraint::XPBDConstraintProjector<ConstraintType>>(_dt, constraint) );

        _constraint_projectors.push_back(std::move(new_projector));
    }

    void solve();

private:
    Real _dt;
    int _num_iter;

    std::vector<std::unique_ptr<Constraint::XPBDConstraintProjector_Base>> _constraint_projectors;
    XPBDConstraints_ConstPtrContainer _constraints;

};
    
} // namespace Solver