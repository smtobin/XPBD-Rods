#include "solver/GaussSeidelSolver.hpp"

namespace Solver
{

GaussSeidelSolver::GaussSeidelSolver(Real dt, int num_iter)
    : _dt(dt), _num_iter(num_iter)
{

}

void GaussSiedelSolver::solve()
{
    for (auto& projector : _constraint_projectors)
    {
        projector.project();
    }
}

} // namespace Solver