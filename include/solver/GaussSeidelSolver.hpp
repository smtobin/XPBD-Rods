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
    void addConstraint(const ConstraintType* constraint, bool use_muller2020_projectors=false)
    {
        /** TODO: Probably need constraint reference or something here. When std::vector reallocates, the pointer to the constraint used by the
         * constraint projector will become invalid, leading to segfault.
         */
        _constraints.template push_back<const ConstraintType*>(constraint);
        if (use_muller2020_projectors)
        {
            _muller2020_constraint_projectors.template emplace_back<Constraint::Muller2020ConstraintProjector<ConstraintType>>(_dt, constraint);
        }
        else
        {
            _constraint_projectors.template emplace_back<Constraint::XPBDConstraintProjector<ConstraintType>>(_dt, constraint);
        }
        
    }

    void solve();

private:
    Real _dt;
    int _num_iter;

    XPBDConstraintProjectors_Container _constraint_projectors;
    Muller2020ConstraintProjectors_Container _muller2020_constraint_projectors;
    XPBDConstraints_ConstPtrContainer _constraints;

};
    
} // namespace Solver