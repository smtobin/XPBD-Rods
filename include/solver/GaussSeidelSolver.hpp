#pragma once

#include "common/constraint_projector_containers.hpp"

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
    void addConstraint(const ConstVectorHandle<ConstraintType>& constraint, Config::XPBDObjectConfig::ProjectorType projector_type = Config::XPBDObjectConfig::ProjectorType::BLOCK)
    {
        /** TODO: Probably need constraint reference or something here. When std::vector reallocates, the pointer to the constraint used by the
         * constraint projector will become invalid, leading to segfault.
         */
        _constraints.template push_back<ConstVectorHandle<ConstraintType>>(constraint);
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

    const XPBDConstraintProjectors_Container& constraintProjectors() const { return _constraint_projectors; }
    const XPBDSeparateConstraintProjectors_Container& separateConstraintProjectors() const { return _separate_constraint_projectors; }
    const Muller2020ConstraintProjectors_Container& muller2020ConstraintProjectors() const { return _muller2020_constraint_projectors; }

private:
    Real _dt;
    int _num_iter;

    XPBDConstraintProjectors_Container _constraint_projectors;
    XPBDSeparateConstraintProjectors_Container _separate_constraint_projectors;
    Muller2020ConstraintProjectors_Container _muller2020_constraint_projectors;

    XPBDConstraints_ConstVectorHandleContainer _constraints;

};
    
} // namespace Solver