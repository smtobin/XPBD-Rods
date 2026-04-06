#include "constraint/FixedParticleConstraint.hpp"

namespace Constraint
{

OneSidedFixedParticleConstraint::OneSidedFixedParticleConstraint(
    const Vec3r& ref_position,
    SimObject::Particle* particle, const Vec3r& r1,
    const AlphaVecType& alpha
)
    : XPBDConstraint<3, 0, 1>({particle}, alpha),
    _ref_position(ref_position), _r1(r1)
{

}

typename OneSidedFixedParticleConstraint::ConstraintVecType OneSidedFixedParticleConstraint::evaluate() const
{
    return (_particles[0]->position + _r1) - _ref_position;
}

typename OneSidedFixedParticleConstraint::GradientMatType OneSidedFixedParticleConstraint::gradient() const
{
    return Mat3r::Identity();
}

} // namespace Constraint
