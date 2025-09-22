#pragma once

#include "common/common.hpp"
#include "common/VariadicVectorContainer.hpp"

#include "constraint/AttachmentConstraint.hpp"
#include "constraint/RodElasticConstraint.hpp"
#include "constraint/XPBDConstraintProjector.hpp"

#include <memory>
#include <vector>

namespace Solver
{

class GaussSeidelSolver
{
public:
    GaussSeidelSolver(Real dt, int num_iter);

    template <typename ConstraintType, typename... Args>
    void addConstraint(Args&&... args)
    {
        /** TODO: Probably need constraint reference or something here. When std::vector reallocates, the pointer to the constraint used by the
         * constraint projector will become invalid, leading to segfault.
         */
        _constraints.template emplace_back<ConstraintType>(std::forward<Args>(args)...);

        std::unique_ptr<Constraint::XPBDConstraintProjector_Base> new_projector =
             std::make_unique<Constraint::XPBDConstraintProjector<ConstraintType>>(_dt, &_constraints.template get<ConstraintType>().back());

        _constraint_projectors.push_back(std::move(new_projector));
    }

    void solve();

private:
    Real _dt;
    int _num_iter;

    std::vector<std::unique_ptr<Constraint::XPBDConstraintProjector_Base>> _constraint_projectors;
    VariadicVectorContainer<Constraint::AttachmentConstraint, Constraint::RodElasticConstraint> _constraints;

};
    
} // namespace Solver