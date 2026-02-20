#include "solver/GaussSeidelSolver.hpp"

namespace Solver
{

GaussSeidelSolver::GaussSeidelSolver(Real dt, int num_iter)
    : _dt(dt), _num_iter(num_iter)
{

}

void GaussSeidelSolver::solve(bool initialize)
{
    if (initialize)
    {
        _constraint_projectors.for_each_element([&](auto& projector) {
            projector.initialize();
        });

        _separate_constraint_projectors.for_each_element([&](auto& projector) {
            projector.initialize();
        });

        _muller2020_constraint_projectors.for_each_element([&](auto& projector) {
            projector.initialize();
        });
    }

    for (int gi = 0; gi < _num_iter; gi++)
    {
        _constraint_projectors.for_each_element([&](auto& projector) {
            projector.project();
        });

        _separate_constraint_projectors.for_each_element([&](auto& projector) {
            projector.project();
        });

        _muller2020_constraint_projectors.for_each_element([&](auto& projector) {
            projector.project();
        });
    }
}

template <typename ConstraintList>
struct XPBDConstraintProjectorsTypeListFromConstraintTypeList;

template <typename ...Constraints>
struct XPBDConstraintProjectorsTypeListFromConstraintTypeList<TypeList<Constraints...>>
{
    using type_list = TypeList<Constraint::XPBDConstraintProjector<Constraints>...>;
};

void GaussSeidelSolver::applyFriction()
{
    using CollisionProjectorsTypeList = XPBDConstraintProjectorsTypeListFromConstraintTypeList<XPBDCollisionConstraints_TypeList>::type_list;
    _constraint_projectors.for_each_element(CollisionProjectorsTypeList{}, [&](auto& projector)
    {
        Real lambda = projector.lambda()[0];   // collision constraints should only have 1 constraint (and 1 lambda)
        projector.constraint()->applyFriction(lambda, 0.5, 0.2);
    });
}

} // namespace Solver