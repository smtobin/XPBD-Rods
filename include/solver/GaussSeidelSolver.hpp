#pragma once

#include "common/common.hpp"

#include "constraint/Constraint.hpp"
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
        std::unique_ptr<ConstraintType> new_constraint = std::make_unique<ConstraintType>(std::forward<Args>(args)...);
        std::unique_ptr<Constraint::XPBDConstraintProjector_Base> new_projector =
             std::make_unique<Constraint::XPBDConstraintProjector<ConstraintType>>(_dt);
        _constraints.push_back(std::move(new_constraint));
        _constraint_projectors.push_back(std::move(new_projector));
    }

    void solve();

private:
    Real _dt;
    int _num_iter;

    std::vector<std::unique_ptr<Constraint::XPBDConstraintProjector_Base>> _constraint_projectors;
    std::vector<std::unique_ptr<Constraint::XPBDConstraint>> _constraints;

};
    
} // namespace Solver