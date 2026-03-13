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
    : XPBDConstraint<1, Order1+1 + Order2+1>(concat_arrays(element1->nodes(), element2->nodes()),
     1e-10*AlphaVecType::Ones()),
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
    return 0.1*C;
}

template <int Order1, int Order2>
typename RodRodCollisionConstraint<Order1, Order2>::GradientMatType
RodRodCollisionConstraint<Order1, Order2>::gradient(bool /*update_cache*/) const
{
    GradientMatType grad;
    grad.template block<1, 6*(Order1+1)>(0,0) = -_n.transpose() * _element1->contactPointGradient(_s_hat1, _cp_local1);
    grad.template block<1, 6*(Order2+1)>(0,6*(Order1+1)) = _n.transpose() * _element2->contactPointGradient(_s_hat2, _cp_local2);
    return grad;
}

template <int Order1, int Order2>
typename RodRodCollisionConstraint<Order1, Order2>::SingleParticleGradientMatType
RodRodCollisionConstraint<Order1, Order2>::singleParticleGradient(const SimObject::OrientedParticle* /*node_ptr*/, bool /*use_cache*/) const
{
    throw std::runtime_error("singleParticleGradient not implemented for RodRodCollisionConstraint");
    return SingleParticleGradientMatType::Zero();
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
    for (int i = 0; i < NumParticles; i++)
    {
        SingleParticleGradientMatType particle_i_grad = delC.template block<ConstraintDim, 6>(0, 6*i);
        const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam_tan;
        // std::cout << "    Applying position update to particle " << i << ": " << position_update.transpose() << std::endl;
        _particles[i]->positionUpdate(position_update);
    }
}

template class RodRodCollisionConstraint<1,1>;
template class RodRodCollisionConstraint<1,2>;
template class RodRodCollisionConstraint<2,1>;
template class RodRodCollisionConstraint<2,2>;

} // namespace Constraint