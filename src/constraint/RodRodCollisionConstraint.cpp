#include "constraint/RodRodCollisionConstraint.hpp"
#include "simobject/rod/XPBDRod.hpp"

namespace Constraint
{

RodRodCollisionConstraint::RodRodCollisionConstraint(
    SimObject::OrientedParticle* segment1_particle1, SimObject::OrientedParticle* segment1_particle2,
    Real beta1, Vec3r cp_local1,
    SimObject::OrientedParticle* segment2_particle1, SimObject::OrientedParticle* segment2_particle2,
    Real beta2, Vec3r cp_local2,
    const Vec3r& n, Real mu_s, Real mu_d
)
    : XPBDConstraint<1, 4>({segment1_particle1, segment1_particle2, segment2_particle1, segment2_particle2},
     1e-10*AlphaVecType::Ones()),
    _beta1(beta1), _beta2(beta2),
     _cp_local1(cp_local1), _cp_local2(cp_local2), _n(n), _mu_s(mu_s), _mu_d(mu_d)
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

void RodRodCollisionConstraint::applyFriction(Real lambda_n) const
{
    // return;
    // contact point on rod 1
    const Vec3r R_diff1 = Math::Minus_SO3(_particles[1]->orientation, _particles[0]->orientation);
    const Mat3r exp_Rdiff1 = Math::Exp_so3(_beta1*R_diff1);
    const Mat3r R_local1 = _particles[0]->orientation * exp_Rdiff1;
    const Vec3r cp1 = (1-_beta1)*_particles[0]->position + _beta1*_particles[1]->position + R_local1*_cp_local1;

    // contact point on rod 2
    const Vec3r R_diff2 = Math::Minus_SO3(_particles[3]->orientation, _particles[2]->orientation);
    const Mat3r exp_Rdiff2 = Math::Exp_so3(_beta2*R_diff2);
    const Mat3r R_local2 = _particles[2]->orientation * exp_Rdiff2;
    const Vec3r cp2 = (1-_beta2)*_particles[2]->position + _beta2*_particles[3]->position + R_local2*_cp_local2;


    // previous location of contact point on rigid body 1
    const Vec3r R_diff_prev1 = Math::Minus_SO3(_particles[1]->prev_orientation, _particles[0]->prev_orientation);
    const Mat3r R_local_prev1 = Math::Plus_SO3(_particles[0]->prev_orientation, _beta1*R_diff_prev1);
    const Vec3r prev_cp1 = (1-_beta1)*_particles[0]->prev_position + _beta1*_particles[1]->prev_position + R_local_prev1*_cp_local1;
    // previous location of contact point on rigid body 2
    const Vec3r R_diff_prev2 = Math::Minus_SO3(_particles[3]->prev_orientation, _particles[2]->prev_orientation);
    const Mat3r R_local_prev2 = Math::Plus_SO3(_particles[2]->prev_orientation, _beta2*R_diff_prev2);
    const Vec3r prev_cp2 = (1-_beta2)*_particles[2]->prev_position + _beta2*_particles[3]->prev_position + R_local_prev2*_cp_local2;

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = (cp2 - prev_cp2) - (cp1 - prev_cp1);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;


    // gradients w.r.t. positions of rod 1
    Vec3r tan_dir = dp_tan.normalized();
    Real C = dp_tan.norm();


    Vec3r dC_dp1 = -(1-_beta1)*tan_dir;
    Vec3r dC_dp2 = -_beta1*tan_dir;

    // gradients w.r.t. orientations of rod 1
    const Mat3r gam1 = Math::ExpMap_RightJacobian(_beta1*R_diff1);
    const Mat3r inv_gam1 = Math::ExpMap_InvRightJacobian(R_diff1);
    const Mat3r beta_gam_inv_gam1 = _beta1 * gam1 * inv_gam1;
    const Mat3r R_local_skew_cp1 = _particles[0]->orientation * exp_Rdiff1 * Math::Skew3(_cp_local1);

    const Mat3r dRcp_dR1 = R_local_skew_cp1 * (exp_Rdiff1 - beta_gam_inv_gam1);
    const Mat3r dRcp_dR2 = R_local_skew_cp1 * beta_gam_inv_gam1;

    const Vec3r dC_dR1 = tan_dir.transpose() * dRcp_dR1;
    const Vec3r dC_dR2 = tan_dir.transpose() * dRcp_dR2;

    // gradients w.r.t. positions of rod 2
    Vec3r dC_dp3 = (1-_beta2)*tan_dir;
    Vec3r dC_dp4 = (_beta2)*tan_dir;

    // gradients w.r.t. orientations of rod 1
    const Mat3r gam2 = Math::ExpMap_RightJacobian(_beta2*R_diff2);
    const Mat3r inv_gam2 = Math::ExpMap_InvRightJacobian(R_diff2);
    const Mat3r beta_gam_inv_gam2 = _beta2 * gam2 * inv_gam2;
    const Mat3r R_local_skew_cp2 = _particles[2]->orientation * exp_Rdiff2 * Math::Skew3(_cp_local2);

    const Mat3r dRcp_dR3 = R_local_skew_cp2 * (exp_Rdiff2 - beta_gam_inv_gam2);
    const Mat3r dRcp_dR4 = R_local_skew_cp2 * beta_gam_inv_gam2;

    const Vec3r dC_dR3 = -tan_dir.transpose() * dRcp_dR3;
    const Vec3r dC_dR4 = -tan_dir.transpose() * dRcp_dR4;

    // compute lambda as if we are undoing the entire movement in the tangent direction
    GradientMatType delC;
    delC.block<1,3>(0,0) = dC_dp1;
    delC.block<1,3>(0,3) = dC_dR1;
    delC.block<1,3>(0,6) = dC_dp2;
    delC.block<1,3>(0,9) = dC_dR2;
    delC.block<1,3>(0,12) = dC_dp3;
    delC.block<1,3>(0,15) = dC_dR3;
    delC.block<1,3>(0,18) = dC_dp4;
    delC.block<1,3>(0,21) = dC_dR4;

    // std::cout << std::endl;
    // std::cout << "  p1: " << _particles[0]->position.transpose() << std::endl;
    // std::cout << "  p2: " << _particles[1]->position.transpose() << std::endl;
    // std::cout << "  p3: " << _particles[2]->position.transpose() << std::endl;
    // std::cout << "  p4: " << _particles[3]->position.transpose() << std::endl;

    // std::cout << "  dp_tan: " << dp_tan.transpose() << std::endl;
    // std::cout << "  ||dp_tan||: " << dp_tan.norm() << std::endl;
    // std::cout << "  Tangential dir: " << tan_dir.transpose() << std::endl;
    

    Eigen::Vector<Real, StateDim> inertia_inverse;
    for (int i = 0; i < NumParticles; i++)
    {
        inertia_inverse.template block<6,1>(6*i, 0) = 
            Vec6r(1/_particles[i]->mass, 1/_particles[i]->mass, 1/_particles[i]->mass,
                 1/_particles[i]->Ib[0], 1/_particles[i]->Ib[1], 1/_particles[i]->Ib[2]);
    }

    Real LHS = delC * inertia_inverse.asDiagonal() * delC.transpose();
    Real dlam_tan = -C / LHS;

    // std::cout << "  Nominal dlam_tan: " << dlam_tan << std::endl;
    // std::cout << "  lambda_n: " << lambda_n << std::endl;

    

    // get last relative velocity between contact points, in the tangential direction
    Vec3r v_cp1 = (1-_beta1)*_particles[0]->lin_velocity + _beta1*_particles[1]->lin_velocity + dRcp_dR1*_particles[0]->ang_velocity + dRcp_dR2*_particles[1]->ang_velocity;
    Vec3r v_cp2 = (1-_beta2)*_particles[2]->lin_velocity + _beta2*_particles[3]->lin_velocity + dRcp_dR3*_particles[2]->ang_velocity + dRcp_dR4*_particles[3]->ang_velocity;
    Vec3r v_rel = v_cp1 - v_cp2;
    Vec3r v_rel_tan = v_rel - (v_rel.dot(_n))*_n;

    // determine if static friction should be applied (i.e. dp_tan should be reduced to 0)
    // if relative velocity between contact points in the tangential ~~ 0, then the bodies were static relative to one another last step
    if (v_rel_tan.norm() < 1e-4)
    {
        // if nominal tangent lambda >= coeff of static friction * normal lambda, then dynamic friction should be applied
        // so clamp the tangent lambda to correspond to the dynamic force
        if (dlam_tan >= _mu_s*lambda_n)
            dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
        // else
            // std::cout << "STATIC FRICTION" << std::endl;
    }
    // bodies were moving relative to each other last step, so dynamic friction should be applied
    else
    {
        // clamp the tangent lambda to correspond to the dynamic friction force
        dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
    }

    // std::cout << "  Clamped dlam_tan: " << dlam_tan << std::endl;

    // update nodes
    for (int i = 0; i < NumParticles; i++)
    {
        SingleParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam_tan;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _particles[i]->positionUpdate(position_update);
    }
}

} // namespace Constraint