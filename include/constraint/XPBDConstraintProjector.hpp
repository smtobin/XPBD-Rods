#pragma once

#include "common/common.hpp"
#include "simobject/OrientedParticle.hpp"

namespace Constraint
{

class XPBDConstraintProjector_Base
{
public:
    XPBDConstraintProjector_Base(Real dt)
        : _dt(dt)
    {}

    void initialize()
    {
        _lambda = 0;
    }

    virtual void project() = 0;

protected:
    Real _dt;
};

template <typename Constraint>
class XPBDConstraintProjector : XPBDConstraintProjector_Base
{
public:
    XPBDConstraintProjector(Real dt, Constraint* constraint)
        : XPBDConstraintProjector_Base(dt)
    {
        _constraint(constraint);
    }

    virtual void project() override
    {
        typename Constraint::ConstraintVecType C = _constraint->evaluate();
        typename Constraint::GradientMatType delC = _constraint->gradient(true);

        const typename Constraint::ConstraintVecType RHS = -C - _constraint->alpha().asDiagonal() * _lambda;

        Eigen::Vector<Real, Constraint::StateDim> inertia_inverse;
        for (const auto& particle : _constraint->particles())
        {
            inertia_inverse << 1/particle->mass, 1/particle->mass, 1/particle->mass;
            inertia_inverse << 1/particle->Ib[0], 1/particle->Ib[1], 1/particle->Ib[2];
        }

        Eigen::Matrix<Real, Constraint::ConstraintDim, Constraint::ConstraintDim> LHS =
            delC * inertia_inverse.asDiagonal() * delC.transpose() + _constraint->alpha().asDiagonal();

        const typename Constraint::ConstraintVecType dlam = LHS.llt().matrixL().solve(RHS);
        _lambda += dlam;

        // update nodes
        for (int i = 0; i < Constraint::NumParticles; i++)
        {
            SimObject::OrientedParticle* particle_i = _constraint->particles()[i];
            const Vec6r position_update = inertia_inverse.block<6,1>(6*i, 0).asDiagonal() * constraint->singleParticleGradient(particle_i, true).transpose() * dlam;
            particle_i->positionUpdate(position_update);
        }
    }
private:
    typename Constraint::ConstraintVecType _lambda;
    Constraint* _constraint;
}

} // namespace Constraint