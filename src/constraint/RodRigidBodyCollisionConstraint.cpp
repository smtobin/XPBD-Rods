#include "constraint/RodRigidBodyCollisionConstraint.hpp"

#include "common/ArrayConcatenate.hpp"

namespace Constraint
{

template <int Order>
RodRigidBodyCollisionConstraint<Order>::RodRigidBodyCollisionConstraint(
    SimObject::RodElement<Order>* element,
    Real s_hat, const Vec3r& cp_local_rod,
    SimObject::OrientedParticle* com_rb, const Vec3r& cp_local_rb,
    const Vec3r& n,
    Real mu_s, Real mu_d
)
    : XPBDConstraint<1, 1 + Order+1, 0>(concat_arrays(element->nodes(), std::array<SimObject::OrientedParticle*,1>{com_rb}), 1.0e-10*AlphaVecType::Ones()), 
    _element(element), _s_hat(s_hat), _cp_local_rod(cp_local_rod),  _cp_local_rb(cp_local_rb), _n(n), _mu_s(mu_s), _mu_d(mu_d)
{
}

template <int Order>
typename RodRigidBodyCollisionConstraint<Order>::ConstraintVecType RodRigidBodyCollisionConstraint<Order>::evaluate() const
{
    // contact point on rod
    Vec3r cp_rod = _element->contactPoint(_s_hat, _cp_local_rod);

    // contact point on rigid body
    const Vec3r cp_rb = _oriented_particles[2]->position + _oriented_particles[2]->orientation *  _cp_local_rb;

    ConstraintVecType C;
    C[0] = (cp_rb - cp_rod).dot(_n);
    return 0.5*C;
}

template <int Order>
typename RodRigidBodyCollisionConstraint<Order>::GradientMatType RodRigidBodyCollisionConstraint<Order>::gradient() const
{
    // gradients w.r.t. rigid body
    Vec3r dC_dprb = _n;
    Vec3r dC_dRrb = -_n.transpose() * _oriented_particles[2]->orientation * Math::Skew3( _cp_local_rb);

    GradientMatType grad;
    grad.template block<1, 6*(Order+1)>(0,0) = -_n.transpose() * _element->contactPointGradient(_s_hat, _cp_local_rod);
    grad.template block<1,3>(0, 6*(Order+1)) = dC_dprb;
    grad.template block<1,3>(0,6*(Order+1)+3) = dC_dRrb;

    return grad;
}

template <int Order>
void RodRigidBodyCollisionConstraint<Order>::applyFriction(Real lambda_n) const
{
    // contact point on rod
    Vec3r cp_rod = _element->contactPoint(_s_hat, _cp_local_rod);

    // previous location of contact point on rod
    Vec3r prev_cp_rod = _element->previousContactPoint(_s_hat, _cp_local_rod);

    // contact point on rigid body
    const Vec3r cp_rb = _oriented_particles[2]->position + _oriented_particles[2]->orientation * _cp_local_rb;

    // previous location of contact point on rigid body 1
    const Vec3r prev_cp_rb = _oriented_particles[2]->prev_position + _oriented_particles[2]->prev_orientation * _cp_local_rb;

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = (cp_rb - prev_cp_rb) - (cp_rod - prev_cp_rod);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;

    // compute lambda as if we are undoing the entire movement in the tangent direction
    Vec3r tan_dir = dp_tan.normalized();
    Real C = dp_tan.norm();

    const Vec3r dC_dp3 = tan_dir;
    const Vec3r dC_dR3 = -tan_dir.transpose() * _oriented_particles[0]->orientation * Math::Skew3(_cp_local_rb);

    GradientMatType delC;
    delC.template block<1, 6*(Order+1)>(0,0) = -tan_dir.transpose() * _element->contactPointGradient(_s_hat, _cp_local_rod);
    delC.template block<1,3>(0,6*(Order+1)) = dC_dp3;
    delC.template block<1,3>(0,6*(Order+1)+3) = dC_dR3;

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
    Vec3r v_cp_rod = _element->contactPointVelocity(_s_hat, _cp_local_rod);
    Vec3r v_cp_rb = _oriented_particles[2]->lin_velocity + _oriented_particles[2]->orientation * Math::Skew3(_oriented_particles[2]->ang_velocity) * _cp_local_rb;
    Vec3r v_rel = v_cp_rb - v_cp_rod;
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <int Order>
OneSidedRodRigidBodyCollisionConstraint<Order>::OneSidedRodRigidBodyCollisionConstraint(
    SimObject::RodElement<Order>* element,
    Real s_hat, const Vec3r& cp_local_rod,
    const Vec3r& rb_cp,
    const Vec3r& n,
    Real mu_s, Real mu_d
)
    : XPBDConstraint<1, Order+1, 0>(element->nodes(), 1.0e-10*AlphaVecType::Ones()),
     _element(element), _s_hat(s_hat), _cp_local_rod(cp_local_rod), _cp(rb_cp), _n(n), _mu_s(mu_s), _mu_d(mu_d)
{
}

template <int Order>
typename OneSidedRodRigidBodyCollisionConstraint<Order>::ConstraintVecType OneSidedRodRigidBodyCollisionConstraint<Order>::evaluate() const
{
    // contact point on rod
    Vec3r cp_rod = _element->contactPoint(_s_hat, _cp_local_rod);

    ConstraintVecType C;
    C[0] = (_cp - cp_rod).dot(_n);
    return 0.5*C;
}

template <int Order>
typename OneSidedRodRigidBodyCollisionConstraint<Order>::GradientMatType OneSidedRodRigidBodyCollisionConstraint<Order>::gradient() const
{
    GradientMatType grad = -_n.transpose() * _element->contactPointGradient(_s_hat, _cp_local_rod);
    return grad;
}

template <int Order>
void OneSidedRodRigidBodyCollisionConstraint<Order>::applyFriction(Real lambda_n) const
{
    // contact point on rod
    Vec3r cp_rod = _element->contactPoint(_s_hat, _cp_local_rod);

    // previous location of contact point on rod
    Vec3r prev_cp_rod = _element->previousContactPoint(_s_hat, _cp_local_rod);

    // get tangent direction (direction of relative motion, with normal component removed)
    Vec3r dp_rel = -(cp_rod - prev_cp_rod);
    Vec3r dp_tan = dp_rel - (dp_rel.dot(_n))*_n;

    if (dp_tan.norm() < 1e-8)
        return;


    // gradients w.r.t. positions of rod 1
    Vec3r tan_dir = dp_tan.normalized();
     Real C = dp_tan.norm();

    // compute lambda as if we are undoing the entire movement in the tangent direction
    GradientMatType delC = -tan_dir.transpose() * _element->contactPointGradient(_s_hat, _cp_local_rod);

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
    Vec3r v_cp_rod = _element->contactPointVelocity(_s_hat, _cp_local_rod);
    Vec3r v_rel = v_cp_rod;
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

template class RodRigidBodyCollisionConstraint<1>;
template class RodRigidBodyCollisionConstraint<2>;
template class RodRigidBodyCollisionConstraint<3>;

template class OneSidedRodRigidBodyCollisionConstraint<1>;
template class OneSidedRodRigidBodyCollisionConstraint<2>;
template class OneSidedRodRigidBodyCollisionConstraint<3>;

} // namespace Constraint