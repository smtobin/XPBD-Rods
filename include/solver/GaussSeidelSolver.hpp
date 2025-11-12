#pragma once

#include "common/common.hpp"
#include "common/VariadicVectorContainer.hpp"

#include "constraint/FixedJointConstraint.hpp"
#include "constraint/RodElasticConstraint.hpp"
#include "constraint/PointLineConstraint.hpp"
#include "constraint/XPBDConstraintProjector.hpp"
#include "constraint/XPBDSeparateConstraintProjector.hpp"
#include "constraint/muller2020/MullerConstraintProjector.hpp"

#include "config/XPBDObjectConfig.hpp"
#include <memory>
#include <vector>

namespace Solver
{

class GaussSeidelSolver
{
public:
    GaussSeidelSolver(Real dt, int num_iter);

    template <typename ConstraintType>
    void addConstraint(const ConstraintType* constraint, Config::XPBDObjectConfig::ProjectorType projector_type = Config::XPBDObjectConfig::ProjectorType::BLOCK)
    {
        /** TODO: Probably need constraint reference or something here. When std::vector reallocates, the pointer to the constraint used by the
         * constraint projector will become invalid, leading to segfault.
         */
        _constraints.template push_back<const ConstraintType*>(constraint);
        if (projector_type == Config::XPBDObjectConfig::ProjectorType::MULLER2020)
        {
            _muller2020_constraint_projectors.template emplace_back<Constraint::Muller2020ConstraintProjector<ConstraintType>>(_dt, constraint);
        }
        else if (projector_type == Config::XPBDObjectConfig::ProjectorType::BLOCK)
        {
            _constraint_projectors.template emplace_back<Constraint::XPBDConstraintProjector<ConstraintType>>(_dt, constraint);
        }
        else if (projector_type == Config::XPBDObjectConfig::ProjectorType::SEPARATE)
        {
            _separate_constraint_projectors.template emplace_back<Constraint::XPBDSeparateConstraintProjector<ConstraintType>>(_dt, constraint);
        }
    }

    void solve();

private:
    Real _dt;
    int _num_iter;

    XPBDConstraintProjectors_Container _constraint_projectors;
    XPBDSeparateConstraintProjectors_Container _separate_constraint_projectors;
    Muller2020ConstraintProjectors_Container _muller2020_constraint_projectors;
    XPBDConstraints_ConstPtrContainer _constraints;

};
    
} // namespace Solver