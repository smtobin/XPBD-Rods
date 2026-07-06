#include "constraint/CoordinateConstraint.hpp"

namespace Constraint
{

OneSidedCoordinateConstraint::OneSidedCoordinateConstraint(
    SimObject::OrientedParticle* particle1,
    const Vec3r& fixed_pos, int c_index
)
    : XPBDConstraint<1,1,0>({particle1}, AlphaVecType::Zero()), _fixed_pos(fixed_pos), _c_index(c_index)
{
}

OneSidedCoordinateConstraint::ConstraintVecType OneSidedCoordinateConstraint::evaluate() const
{
    OneSidedCoordinateConstraint::ConstraintVecType C_vec;
    C_vec[0] = _oriented_particles[0]->position[_c_index] - _fixed_pos[_c_index];
    return C_vec;
}

OneSidedCoordinateConstraint::GradientMatType OneSidedCoordinateConstraint::gradient() const
{
    GradientMatType grad;
    // gradients of positional constraints
    const Mat3r dCp_dp1 = Mat3r::Identity();
    const Mat3r dCp_dor1 = Mat3r::Zero();

    grad.block<1,3>(0,0) = dCp_dp1.row(_c_index);
    grad.block<1,3>(0,3) = dCp_dor1.row(_c_index);

    return grad;
}

} // namespace Constraint