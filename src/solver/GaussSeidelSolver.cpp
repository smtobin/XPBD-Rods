#include "solver/GaussSeidelSolver.hpp"

namespace Solver
{

GaussSeidelSolver::GaussSeidelSolver(Real dt, int num_iter)
    : _dt(dt), _num_iter(num_iter)
{

}

void GaussSeidelSolver::solve()
{
    _constraint_projectors.for_each_element([&](auto& projector) {
        projector.initialize();
    });

    _muller2020_constraint_projectors.for_each_element([&](auto& projector) {
        projector.initialize();
    });

    for (int gi = 0; gi < _num_iter; gi++)
    {
        _constraint_projectors.for_each_element([&](auto& projector) {
            projector.project();
        });

        _muller2020_constraint_projectors.for_each_element([&](auto& projector) {
            projector.project();
        });
    }
}

} // namespace Solver