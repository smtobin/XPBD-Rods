#include "constraint/RodRodCollisionConstraint.hpp"
#include "simobject/rod/XPBDRod.hpp"

namespace Constraint
{

RodRodCollisionConstraint::RodRodCollisionConstraint(
        SimObject::XPBDRodSegment segment1,
        Real beta1, Vec3r cp_local1,
        SimObject::XPBDRodSegment segment2,
        Real beta2, Vec3r cp_local2,
        const Vec3r& n
)
    : XPBDConstraint<1, 4>({segment1.particle1(), segment1.particle2(), segment2.particle1(), segment2.particle2()},
     1e-10*AlphaVecType::Ones()),
     _segment1(segment1), _segment2(segment2), _beta1(beta1), _beta2(beta2),
     _cp_local1(cp_local1), _cp_local2(cp_local2), _n(n)
{
}

RodRodCollisionConstraint::ConstraintVecType RodRodCollisionConstraint::evaluate() const
{
    // contact point on rod 1
    const Vec3r R_diff1 = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r R_local1 = Math::Plus_SO3(_particles[0]->orientation, _beta1*R_diff1);
    const Vec3r cp_rod1 = (1-_beta1)*_particles[0]->position + _beta1*_particles[1]->position + R_local1*_cp_local1;

    // contact point on rod 2
    const Vec3r R_diff2 = Math::Minus_SO3(_particles[3]->orientation, _particles[2]->orientation);
    const Mat3r R_local2 = Math::Plus_SO3(_particles[2]->orientation, _beta2*R_diff2);
    const Vec3r cp_rod2 = (1-_beta2)*_particles[2]->position + _beta2*_particles[3]->position + R_local2*_cp_local2;

    ConstraintVecType C;
    C[0] = (cp_rod2 - cp_rod1).dot(_n);
    // std::cout << "Nominal C: " << C[0] << std::endl;
    return 0.1*C;
}

RodRodCollisionConstraint::GradientMatType RodRodCollisionConstraint::gradient(bool /*update_cache*/) const
{
    // gradients w.r.t. positions of rod 1
    Vec3r dC_dp1 = -(1-_beta1)*_n;
    Vec3r dC_dp2 = -_beta1*_n;

    // gradients w.r.t. orientations of rod 1
    const Vec3r R_diff1 = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r exp_Rdiff1 = Math::Exp_so3(_beta1*R_diff1);
    const Mat3r gam1 = Math::ExpMap_RightJacobian(_beta1*R_diff1);
    const Mat3r inv_gam1 = Math::ExpMap_InvRightJacobian(R_diff1);
    const Mat3r beta_gam_inv_gam1 = _beta1 * gam1 * inv_gam1;
    const Mat3r R_local_skew_cp1 = _particles[0]->orientation * exp_Rdiff1 * Math::Skew3(_cp_local1);

    const Vec3r dC_dR1 = _n.transpose() * R_local_skew_cp1 * (exp_Rdiff1 - beta_gam_inv_gam1);
    const Vec3r dC_dR2 = _n.transpose() * R_local_skew_cp1 * beta_gam_inv_gam1;

    // gradients w.r.t. positions of rod 2
    Vec3r dC_dp3 = (1-_beta2)*_n;
    Vec3r dC_dp4 = (_beta2)*_n;

    // gradients w.r.t. orientations of rod 1
    const Vec3r R_diff2 = Math::Minus_SO3(_particles[3]->orientation, _particles[2]->orientation);
    const Mat3r exp_Rdiff2 = Math::Exp_so3(_beta2*R_diff2);
    const Mat3r gam2 = Math::ExpMap_RightJacobian(_beta2*R_diff2);
    const Mat3r inv_gam2 = Math::ExpMap_InvRightJacobian(R_diff2);
    const Mat3r beta_gam_inv_gam2 = _beta2 * gam2 * inv_gam2;
    const Mat3r R_local_skew_cp2 = _particles[2]->orientation * exp_Rdiff2 * Math::Skew3(_cp_local2);

    const Vec3r dC_dR3 = -_n.transpose() * R_local_skew_cp2 * (exp_Rdiff2 - beta_gam_inv_gam2);
    const Vec3r dC_dR4 = -_n.transpose() * R_local_skew_cp2 * beta_gam_inv_gam2;

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = dC_dR1;
    grad.block<1,3>(0,6) = dC_dp2;
    grad.block<1,3>(0,9) = dC_dR2;
    grad.block<1,3>(0,12) = dC_dp3;
    grad.block<1,3>(0,15) = dC_dR3;
    grad.block<1,3>(0,18) = dC_dp4;
    grad.block<1,3>(0,21) = dC_dR4;

    return grad;
}

RodRodCollisionConstraint::SingleParticleGradientMatType RodRodCollisionConstraint::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for RodRodCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
}

} // namespace Constraint