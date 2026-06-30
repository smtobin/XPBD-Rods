#include "constraint/RigidBodyCollisionConstraint.hpp"

namespace Constraint
{

RigidBodyCollisionConstraint::RigidBodyCollisionConstraint(
    SimObject::OrientedParticle* com1, const Vec3r& r1,
    SimObject::OrientedParticle* com2, const Vec3r& r2,
    const Vec3r& n,
    Real mu_s, Real mu_d
)
    : XPBDConstraint<1, 2, 0>({com1, com2}, 1.0e-10*AlphaVecType::Ones()), _r1(r1), _r2(r2), _n(n), _mu_s(mu_s), _mu_d(mu_d)
{
}

RigidBodyCollisionConstraint::ConstraintVecType RigidBodyCollisionConstraint::evaluate() const
{
    // contact point on rigid body 1
    const Vec3r cp1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    // contact point on rigid body 2
    const Vec3r cp2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;

    ConstraintVecType C;
    C[0] = (cp2 - cp1).dot(_n);
    return C;
}

RigidBodyCollisionConstraint::GradientMatType RigidBodyCollisionConstraint::gradient() const
{
    // positional gradients
    Vec3r dC_dp1 = -_n;
    Vec3r dC_dp2 = _n;

    // orientation gradients
    Vec3r dC_dR1 = _n.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r1);
    Vec3r dC_dR2 = -_n.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_r2);

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = dC_dR1;
    grad.block<1,3>(0,6) = dC_dp2;
    grad.block<1,3>(0,9) = dC_dR2;

    return grad;
}

void RigidBodyCollisionConstraint::applyFriction(Real lambda_n) const
{
    if (lambda_n <= 0)
        return;
    // std::cout << "\nTwoSided Applying friction! lambda_n=" << lambda_n << " mu_s=" << mu_s << " mu_d=" << mu_d << std::endl;

    // contact point on rigid body 1
    const Vec3r cp1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;
    // contact point on rigid body 2
    const Vec3r cp2 = _oriented_particles[1]->position + _oriented_particles[1]->orientation * _r2;


    // previous location of contact point on rigid body 1
    const Vec3r prev_cp1 = _oriented_particles[0]->prev_position + _oriented_particles[0]->prev_orientation * _r1;
    // previous location of contact point on rigid body 2
    const Vec3r prev_cp2 = _oriented_particles[1]->prev_position + _oriented_particles[1]->prev_orientation * _r2;

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = (cp1 - prev_cp1) - (cp2 - prev_cp2);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;

    // compute lambda as if we are undoing the entire movement in the tangent direction
    Vec3r tan_dir = dp_tan.normalized();
    Real C = dp_tan.norm();//(cp1 - cp2).dot(tan_dir);
    Vec3r dC_dp1 = tan_dir;
    Vec3r dC_dp2 = -tan_dir;
    Vec3r dC_dR1 = -tan_dir.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r1);
    Vec3r dC_dR2 = tan_dir.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_r2);
    GradientMatType delC;
    delC.block<1,3>(0,0) = dC_dp1;
    delC.block<1,3>(0,3) = dC_dR1;
    delC.block<1,3>(0,6) = dC_dp2;
    delC.block<1,3>(0,9) = dC_dR2;

    // std::cout << "  dp_tan: " << dp_tan.transpose() << std::endl;
    // std::cout << "  ||dp_tan||: " << dp_tan.norm() << std::endl;
    // std::cout << "  Tangential dir: " << tan_dir.transpose() << std::endl;
    

    Eigen::Vector<Real, StateDim> inertia_inverse;
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        inertia_inverse.template block<6,1>(6*i, 0) = 
            Vec6r(1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass,
                 1/_oriented_particles[i]->Ib[0], 1/_oriented_particles[i]->Ib[1], 1/_oriented_particles[i]->Ib[2]);
    }

    Real LHS = delC * inertia_inverse.asDiagonal() * delC.transpose();
    Real dlam_tan = -C / LHS;

    // std::cout << "  Nominal dlam_tan: " << dlam_tan << std::endl;

    

    // get last relative velocity between contact points, in the tangential direction
    Vec3r v_cp1 = _oriented_particles[0]->lin_velocity + _oriented_particles[0]->orientation * Math::Skew3(_oriented_particles[0]->ang_velocity) * _r1;
    Vec3r v_cp2 = _oriented_particles[1]->lin_velocity + _oriented_particles[1]->orientation * Math::Skew3(_oriented_particles[1]->ang_velocity) * _r2;
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
        //     std::cout << "STATIC FRICTION" << std::endl;
    }
    // bodies were moving relative to each other last step, so dynamic friction should be applied
    else
    {
        // clamp the tangent lambda to correspond to the dynamic force
        dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
    }

    // std::cout << "  Clamped dlam_tan: " << dlam_tan << std::endl;

    // update nodes
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        using SingleOrientedParticleGradientMatType = Eigen::Matrix<Real, ConstraintDim, 6>;
        SingleOrientedParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam_tan;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _oriented_particles[i]->positionUpdate(position_update);
    }

}

void RigidBodyCollisionConstraint::applyRestitution() const
{
    Real e = 0.0;

    // get previous relative velocity between contact points, in the normal direction
    Vec3r v_cp1_prev = _oriented_particles[0]->prev_lin_velocity + _oriented_particles[0]->prev_orientation * Math::Skew3(_oriented_particles[0]->prev_ang_velocity) * _r1;
    Vec3r v_cp2_prev = _oriented_particles[1]->prev_lin_velocity + _oriented_particles[1]->prev_orientation * Math::Skew3(_oriented_particles[1]->prev_ang_velocity) * _r2;
    Vec3r v_rel_prev = v_cp1_prev - v_cp2_prev;
    
    Real v_norm_mag_prev = _n.dot(v_rel_prev);

    // get current relative velocity between contact points, in the normal direction
    Vec3r v_cp1 = _oriented_particles[0]->lin_velocity + _oriented_particles[0]->orientation * Math::Skew3(_oriented_particles[0]->ang_velocity) * _r1;
    Vec3r v_cp2 = _oriented_particles[1]->lin_velocity + _oriented_particles[1]->orientation * Math::Skew3(_oriented_particles[1]->ang_velocity) * _r2;
    Vec3r v_rel = v_cp1 - v_cp2;
    
    Real v_norm_mag = _n.dot(v_rel);

    // CHECK FOR V_NORM_MAG < 0 HERE ??

    // the new relative velocity in the normal direction should be the previous normal velocity "reflected" to point in the opposite direction, and
    //   scaled by the coefficient of restitution
    Real v_norm_new_mag = std::min(Real(0.0), -e*v_norm_mag_prev);
    if (std::abs(v_norm_new_mag) < 1e-2)
        v_norm_new_mag = 0;

    // velocity-level constraint
    Real C_vel = v_norm_mag - v_norm_new_mag;

    // constraint gradients (with respect to velocity and angular velocity)
    Vec3r dC_dv1 = _n;
    Vec3r dC_dv2 = -_n;
    Vec3r dC_dw1 = -_n.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r1);
    Vec3r dC_dw2 = _n.transpose() * _oriented_particles[1]->orientation * Math::Skew3(_r2);
    GradientMatType delC;
    delC.block<1,3>(0,0) = dC_dv1;
    delC.block<1,3>(0,3) = dC_dw1;
    delC.block<1,3>(0,6) = dC_dv2;
    delC.block<1,3>(0,9) = dC_dw2;

    // construct inertia inverse matrix
    Eigen::Vector<Real, StateDim> inertia_inverse;
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        inertia_inverse.template block<6,1>(6*i, 0) = 
            Vec6r(1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass,
                 1/_oriented_particles[i]->Ib[0], 1/_oriented_particles[i]->Ib[1], 1/_oriented_particles[i]->Ib[2]);
    }

    // assemble LHS (alpha = 0)
    Real LHS = delC * inertia_inverse.asDiagonal() * delC.transpose();

    // solve (alpha = 0)
    Real dlam = -C_vel / LHS;

    // compute velocity update
    Eigen::Vector<Real, StateDim> vel_update = inertia_inverse.asDiagonal() * delC.transpose() * dlam;
    _oriented_particles[0]->lin_velocity += vel_update.block<3,1>(0,0);
    _oriented_particles[0]->ang_velocity += vel_update.block<3,1>(3,0);
    _oriented_particles[1]->lin_velocity += vel_update.block<3,1>(6,0);
    _oriented_particles[1]->ang_velocity += vel_update.block<3,1>(9,0);

}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


OneSidedRigidBodyCollisionConstraint::OneSidedRigidBodyCollisionConstraint(
    const Vec3r& cp,
    SimObject::OrientedParticle* com1, const Vec3r& r1,
    const Vec3r& n,
    Real mu_s, Real mu_d
)
    : XPBDConstraint<1, 1, 0>({com1}, 1.0e-10*AlphaVecType::Ones()), _r1(r1), _cp(cp), _n(n), _mu_s(mu_s), _mu_d(mu_d)
{
}

OneSidedRigidBodyCollisionConstraint::ConstraintVecType OneSidedRigidBodyCollisionConstraint::evaluate() const
{
    // contact point on rigid body 1
    const Vec3r cp1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;

    ConstraintVecType C;
    C[0] = (cp1 - _cp).dot(_n);
    // return 0.1*C;
    return C;
}

OneSidedRigidBodyCollisionConstraint::GradientMatType OneSidedRigidBodyCollisionConstraint::gradient() const
{
    // positional gradients
    Vec3r dC_dp1 = _n;

    // orientation gradients
    Vec3r dC_dR1 = -_n.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r1);

    GradientMatType grad;
    grad.block<1,3>(0,0) = dC_dp1;
    grad.block<1,3>(0,3) = dC_dR1;

    return grad;
}

void OneSidedRigidBodyCollisionConstraint::applyFriction(Real lambda_n) const
{
    if (lambda_n <= 0)
        return;

    // std::cout << "\nOneSided Applying friction! lambda_n=" << lambda_n << " mu_s=" << _mu_s << " mu_d=" << _mu_d << std::endl;
    // contact point on rigid body 1
    const Vec3r cp1 = _oriented_particles[0]->position + _oriented_particles[0]->orientation * _r1;


    // previous location of contact point on rigid body 1
    const Vec3r prev_cp1 = _oriented_particles[0]->prev_position + _oriented_particles[0]->prev_orientation * _r1;

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = (cp1 - prev_cp1);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;

    // compute lambda as if we are undoing the entire movement in the tangent direction
    Vec3r tan_dir = dp_tan.normalized();
    Real C = dp_tan.norm();//(cp1 - _cp).dot(tan_dir);
    Vec3r dC_dp1 = tan_dir;
    Vec3r dC_dR1 = -tan_dir.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r1);
    GradientMatType delC;
    delC.block<1,3>(0,0) = dC_dp1;
    delC.block<1,3>(0,3) = dC_dR1;

    // std::cout << "  dp_tan: " << dp_tan.transpose() << std::endl;
    // std::cout << "  ||dp_tan||: " << dp_tan.norm() << std::endl;
    // std::cout << "  Tangential dir: " << tan_dir.transpose() << std::endl;
    

    Eigen::Vector<Real, StateDim> inertia_inverse;
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        inertia_inverse.template block<6,1>(6*i, 0) = 
            Vec6r(1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass,
                 1/_oriented_particles[i]->Ib[0], 1/_oriented_particles[i]->Ib[1], 1/_oriented_particles[i]->Ib[2]);
    }

    Real LHS = delC * inertia_inverse.asDiagonal() * delC.transpose();
    Real dlam_tan = -C / LHS;

    // std::cout << "  Nominal dlam_tan: " << dlam_tan << std::endl;

    // get last relative velocity between contact points, in the tangential direction
    Vec3r v_cp1 = _oriented_particles[0]->lin_velocity + _oriented_particles[0]->orientation * Math::Skew3(_oriented_particles[0]->ang_velocity) * _r1;
    Vec3r v_rel = v_cp1;
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
        //     std::cout << "STATIC FRICTION!" << std::endl;
    }
    // bodies were moving relative to each other last step, so dynamic friction should be applied
    else
    {
        // clamp the tangent lambda to correspond to the dynamic force
        dlam_tan = std::clamp(dlam_tan, -_mu_d*lambda_n, _mu_d*lambda_n);
    }

    // std::cout << "  Clamped dlam_tan: " << dlam_tan << std::endl;

    // update nodes
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        using SingleOrientedParticleGradientMatType = Eigen::Matrix<Real, ConstraintDim, 6>;
        SingleOrientedParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam_tan;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _oriented_particles[i]->positionUpdate(position_update);
    }

}

void OneSidedRigidBodyCollisionConstraint::applyRestitution() const
{
    Real e = 0.8;

    // get previous relative velocity between contact points, in the normal direction
    Vec3r v_cp1_prev = _oriented_particles[0]->prev_lin_velocity + _oriented_particles[0]->prev_orientation * Math::Skew3(_oriented_particles[0]->prev_ang_velocity) * _r1;
    Vec3r v_rel_prev = -v_cp1_prev; // normal points outward from fixed body, hence the negative sign
    
    Real v_norm_mag_prev = _n.dot(v_rel_prev);

    // get current relative velocity between contact points, in the normal direction
    Vec3r v_cp1 = _oriented_particles[0]->lin_velocity + _oriented_particles[0]->orientation * Math::Skew3(_oriented_particles[0]->ang_velocity) * _r1;
    Vec3r v_rel = -v_cp1;
    
    Real v_norm_mag = _n.dot(v_rel);

    // CHECK FOR V_NORM_MAG < 0 HERE ??

    // std::cout << "v_norm_mag: " << v_norm_mag << std::endl;

    // the new relative velocity in the normal direction should be the previous normal velocity "reflected" to point in the opposite direction, and
    //   scaled by the coefficient of restitution
    Real v_norm_new_mag = std::min(Real(0.0), -e*v_norm_mag_prev);
    if (std::abs(v_norm_new_mag) < 1e-2)
        v_norm_new_mag = 0;

    // std::cout << "v_norm_new_mag: " << v_norm_new_mag << std::endl;

    // velocity-level constraint
    Real C_vel = v_norm_mag - v_norm_new_mag;

    // constraint gradients (with respect to velocity and angular velocity)
    Vec3r dC_dv1 = -_n;
    Vec3r dC_dw1 = _n.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_r1);
    GradientMatType delC;
    delC.block<1,3>(0,0) = dC_dv1;
    delC.block<1,3>(0,3) = dC_dw1;

    // construct inertia inverse matrix
    Eigen::Vector<Real, StateDim> inertia_inverse;
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        inertia_inverse.template block<6,1>(6*i, 0) = 
            Vec6r(1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass, 1/_oriented_particles[i]->mass,
                 1/_oriented_particles[i]->Ib[0], 1/_oriented_particles[i]->Ib[1], 1/_oriented_particles[i]->Ib[2]);
    }

    // assemble LHS (alpha = 0)
    Real LHS = delC * inertia_inverse.asDiagonal() * delC.transpose();

    // solve (alpha = 0)
    Real dlam = -C_vel / LHS;

    // compute velocity update
    Eigen::Vector<Real, StateDim> vel_update = inertia_inverse.asDiagonal() * delC.transpose() * dlam;
    _oriented_particles[0]->lin_velocity += vel_update.block<3,1>(0,0);
    _oriented_particles[0]->ang_velocity += vel_update.block<3,1>(3,0);
}

} // namespace Collision