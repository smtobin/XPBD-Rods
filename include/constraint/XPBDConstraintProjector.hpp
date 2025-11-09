#pragma once

#include "common/common.hpp"
#include "constraint/XPBDConstraintProjector_Base.hpp"
#include "simobject/OrientedParticle.hpp"

namespace Constraint
{

template <typename Constraint>
class XPBDConstraintProjector : public XPBDConstraintProjector_Base
{
public:
    XPBDConstraintProjector(Real dt, const Constraint* constraint)
        : XPBDConstraintProjector_Base(dt), _constraint(constraint)
    {
    }

    virtual void initialize() override
    {
        _lambda = Constraint::ConstraintVecType::Zero();
    }

    virtual void project() override
    {
        typename Constraint::ConstraintVecType C = _constraint->evaluate();
        std::cout << "========\nC: " << C.transpose() << std::endl;
        typename Constraint::GradientMatType delC = _constraint->gradient(true);
        std::cout << "delC:\n" << delC.transpose() << std::endl;

        const typename Constraint::ConstraintVecType RHS = -C - _constraint->alpha().asDiagonal() * _lambda / (_dt*_dt);

        Eigen::Vector<Real, Constraint::StateDim> inertia_inverse;
        // for (const auto& particle : _constraint->particles())
        for (int i = 0; i < Constraint::NumParticles; i++)
        {
            const SimObject::OrientedParticle* particle = _constraint->particles()[i];
            inertia_inverse.template block<6,1>(6*i, 0) = 
                Vec6r(1/particle->mass, 1/particle->mass, 1/particle->mass, 1/particle->Ib[0], 1/particle->Ib[1], 1/particle->Ib[2]);
        }

        Eigen::Matrix<Real, Constraint::ConstraintDim, Constraint::ConstraintDim> LHS =
            delC * inertia_inverse.asDiagonal() * delC.transpose();
        LHS.diagonal() += _constraint->alpha() / (_dt*_dt);
        std::cout << "LHS:\n" << LHS << std::endl;
        std::cout << "RHS:\n" << RHS << std::endl;
        std::cout << "inverse inertia:\n" << inertia_inverse << std::endl;

        const typename Constraint::ConstraintVecType dlam = LHS.llt().solve(RHS);
        _lambda += dlam;

        std::cout << "dlam: " << dlam << std::endl;

        // update nodes
        for (int i = 0; i < Constraint::NumParticles; i++)
        {
            SimObject::OrientedParticle* particle_i = _constraint->particles()[i];
            // std::cout << "Single particle gradient:\n" << _constraint->singleParticleGradient(particle_i, true).transpose() << std::endl;
            const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * _constraint->singleParticleGradient(particle_i, true).transpose() * dlam;
            std::cout << "Position update: " << position_update.transpose() << std::endl;
            particle_i->positionUpdate(position_update);
        }

        typename Constraint::ConstraintVecType newC = _constraint->evaluate();
        std::cout << "New C: " << newC.transpose() << std::endl;
        // assert(0);
    }
private:
    typename Constraint::ConstraintVecType _lambda;
    const Constraint* _constraint;
};

} // namespace Constraint