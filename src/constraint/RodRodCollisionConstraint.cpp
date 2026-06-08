#include "constraint/RodRodCollisionConstraint.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "common/ArrayConcatenate.hpp"

namespace Constraint
{

template <int Order1, int Order2>
RodRodCollisionConstraint<Order1, Order2>::RodRodCollisionConstraint(
    SimObject::RodElement<Order1>* element1,
    Real s_hat1, Vec3r cp_local1,
    SimObject::RodElement<Order2>* element2,
    Real s_hat2, Vec3r cp_local2,
    const Vec3r& n,
    Real mu_s, Real mu_d
)
    : XPBDConstraint<1, Order1+1 + Order2+1,0>(concat_arrays(element1->nodes(), element2->nodes()),
     1e-8*AlphaVecType::Ones()),
    _element1(element1), _element2(element2),
    _s_hat1(s_hat1), _s_hat2(s_hat2),
     _cp_local1(cp_local1), _cp_local2(cp_local2), _n(n), _mu_s(mu_s), _mu_d(mu_d)
{
}

template <int Order1, int Order2>
typename RodRodCollisionConstraint<Order1, Order2>::ConstraintVecType
RodRodCollisionConstraint<Order1, Order2>::evaluate() const
{
    // contact point on rod 1
    Vec3r cp_rod1 = _element1->contactPoint(_s_hat1, _cp_local1);

    // contact point on rod 2
    Vec3r cp_rod2 = _element2->contactPoint(_s_hat2, _cp_local2);

    ConstraintVecType C;
    C[0] = (cp_rod2 - cp_rod1).dot(_n);
    // std::cout << "Nominal C: " << C[0] << std::endl;
    return C;
}

template <int Order1, int Order2>
typename RodRodCollisionConstraint<Order1, Order2>::GradientMatType
RodRodCollisionConstraint<Order1, Order2>::gradient() const
{
    GradientMatType grad;
    grad.template block<1, 6*(Order1+1)>(0,0) = -_n.transpose() * _element1->contactPointGradient(_s_hat1, _cp_local1);
    grad.template block<1, 6*(Order2+1)>(0,6*(Order1+1)) = _n.transpose() * _element2->contactPointGradient(_s_hat2, _cp_local2);
    return grad;
}

template <int Order1, int Order2>
void RodRodCollisionConstraint<Order1, Order2>::applyFriction(Real lambda_n) const
{
    // return;
    // contact point on rod 1
    Vec3r cp1 = _element1->contactPoint(_s_hat1, _cp_local1);

    // contact point on rod 2
    Vec3r cp2 = _element2->contactPoint(_s_hat2, _cp_local2);


    // previous location of contact point on rigid body 1
    Vec3r prev_cp1 = _element1->previousContactPoint(_s_hat1, _cp_local1);

    // previous location of contact point on rigid body 2
    Vec3r prev_cp2 = _element2->previousContactPoint(_s_hat2, _cp_local2);

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = (cp2 - prev_cp2) - (cp1 - prev_cp1);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;


    // gradients w.r.t. positions of rod 1
    Vec3r tan_dir = dp_tan.normalized();
    Real C = dp_tan.norm();


    GradientMatType delC;
    delC.template block<1, 6*(Order1+1)>(0,0) = -tan_dir.transpose() * _element1->contactPointGradient(_s_hat1, _cp_local1);
    delC.template block<1, 6*(Order2+1)>(0,6*(Order1+1)) = tan_dir.transpose() * _element2->contactPointGradient(_s_hat2, _cp_local2);

    // std::cout << std::endl;
    // std::cout << "  p1: " << _oriented_particles[0]->position.transpose() << std::endl;
    // std::cout << "  p2: " << _oriented_particles[1]->position.transpose() << std::endl;
    // std::cout << "  p3: " << _oriented_particles[2]->position.transpose() << std::endl;
    // std::cout << "  p4: " << _oriented_particles[3]->position.transpose() << std::endl;

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
    // std::cout << "  lambda_n: " << lambda_n << std::endl;

    

    // get last relative velocity between contact points, in the tangential direction
    Vec3r v_cp1 = _element1->contactPointVelocity(_s_hat1, _cp_local1);
    Vec3r v_cp2 = _element2->contactPointVelocity(_s_hat2, _cp_local2);
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
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        using SingleOrientedParticleGradientMatType = Eigen::Matrix<Real, ConstraintDim, 6>;
        SingleOrientedParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam_tan;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _oriented_particles[i]->positionUpdate(position_update);
    }
}

template <int Order1, int Order2>
void RodRodCollisionConstraint<Order1, Order2>::applyRestitution() const
{
    Real e = 0.0;

    // get previous relative velocity between contact points, in the normal direction
    Vec3r v_cp_rod_prev1 = _element1->previousContactPointVelocity(_s_hat1, _cp_local1);
    Vec3r v_cp_rod_prev2 = _element2->previousContactPointVelocity(_s_hat2, _cp_local2);
    Vec3r v_rel_prev = v_cp_rod_prev1 - v_cp_rod_prev2;

    Real v_norm_mag_prev = _n.dot(v_rel_prev);

    // std::cout << "\ns hat: " << _s_hat << "cp local: " << _cp_local_rod.transpose() << std::endl;
    // std::cout << "v norm mag prev: " << v_norm_mag_prev << std::endl;


     // get current relative velocity between contact points, in the normal direction
    Vec3r v_cp_rod1 = _element1->contactPointVelocity(_s_hat1, _cp_local1);
    Vec3r v_cp_rod2 = _element2->contactPointVelocity(_s_hat2, _cp_local2);
    Vec3r v_rel = v_cp_rod1 - v_cp_rod2;

    Real v_norm_mag = _n.dot(v_rel);

    std::cout << "v norm mag: " << v_norm_mag << std::endl;

    // CHECK FOR V_NORM_MAG < 0 HERE ??

    // the new relative velocity in the normal direction should be the previous normal velocity "reflected" to point in the opposite direction, and
    //   scaled by the coefficient of restitution
    Real v_norm_new_mag = std::min(Real(0.0), -e*v_norm_mag_prev);
    if (std::abs(v_norm_new_mag) < 1e-2)
        v_norm_new_mag = 0;

    std::cout << "v norm mag new: " << v_norm_new_mag << std::endl;
    // std::cout << "normal: " << _n.transpose() << std::endl;

    // velocity-level constraint
    Real C_vel = v_norm_mag - v_norm_new_mag;

    // constraint gradients (with respect to velocity and angular velocity)

    GradientMatType delC;
    delC.template block<1,6*(Order1+1)>(0,0) = _n.transpose() * _element1->contactPointVelocityGradient(_s_hat1, _cp_local1);
    delC.template block<1,6*(Order2+1)>(0, 6*(Order1+1)) = -_n.transpose() * _element2->contactPointVelocityGradient(_s_hat2, _cp_local2);

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

    // update nodes
    for (int i = 0; i < NumOrientedParticles; i++)
    {
        using SingleOrientedParticleGradientMatType = Eigen::Matrix<Real, ConstraintDim, 6>;
        SingleOrientedParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r velocity_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _oriented_particles[i]->lin_velocity += velocity_update.head<3>();
        _oriented_particles[i]->ang_velocity += velocity_update.tail<3>();
    }

    Vec3r v_cp_rod1_new = _element1->contactPointVelocity(_s_hat1, _cp_local1);
    Vec3r v_cp_rod2_new = _element2->contactPointVelocity(_s_hat2, _cp_local2);
    Vec3r v_rel_new = v_cp_rod1_new - v_cp_rod2_new;

    Real new_v_norm_mag = _n.dot(v_rel_new);

    std::cout << "Desired v norm mag: " << v_norm_new_mag << "  Actual new v norm mag: " << new_v_norm_mag << std::endl;
}

template class RodRodCollisionConstraint<1,1>;
template class RodRodCollisionConstraint<1,2>;
template class RodRodCollisionConstraint<2,2>;
template class RodRodCollisionConstraint<1,3>;
template class RodRodCollisionConstraint<2,3>;
template class RodRodCollisionConstraint<3,3>;

} // namespace Constraint