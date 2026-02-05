#pragma once

#include "common/common.hpp"
#include "constraint/XPBDConstraintProjector_Base.hpp"
#include "simobject/OrientedParticle.hpp"

#include <Eigen/Cholesky>

namespace Constraint
{

template <typename Constraint_>
class XPBDConstraintProjector : public XPBDConstraintProjector_Base
{
public:
    using Constraint = Constraint_;

    XPBDConstraintProjector(Real dt, const ConstVectorHandle<Constraint>& constraint)
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

        // special handling for inequality constraints
        if (_constraint->isInequality())
        {
            // inequality constraints are only enforced when C < 0
            // for vector constraints, set rows of C that are > 0 to 0 so that nothing happens
            // (does this work? I think so. But really XPBDSeparateConstraintProjector should be used instead)
            if constexpr (Constraint::ConstraintDim > 1)
            {
                for (int i = 0; i < Constraint::ConstraintDim; i++)
                {
                    if (C[i] > 0)
                        C[i] = 0;
                }
            }
            // for scalar constraints, just return if C > 0
            else
            {
                if (C[0] > 0)
                    return;
            }
            
        }
        // std::cout << "========\nC: " << C.transpose() << std::endl;
        typename Constraint::GradientMatType delC = _constraint->gradient(true);
        // std::cout << "delC:\n" << delC.transpose() << std::endl;

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
        // std::cout << "LHS:\n" << LHS << std::endl;
        // std::cout << "RHS:\n" << RHS << std::endl;
        // std::cout << "inverse inertia:\n" << inertia_inverse << std::endl;

        const typename Constraint::ConstraintVecType dlam = LHS.llt().solve(RHS);
        _lambda += dlam;

        // std::cout << "dlam: " << dlam << std::endl;

        // update nodes
        for (int i = 0; i < Constraint::NumParticles; i++)
        {
            typename Constraint::SingleParticleGradientMatType particle_i_grad = delC.template block<Constraint::ConstraintDim, 6>(0, 6*i);
            SimObject::OrientedParticle* particle_i = _constraint->particles()[i];
            // std::cout << "Single particle gradient:\n" << _constraint->singleParticleGradient(particle_i, true).transpose() << std::endl;
            const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam;
            // std::cout << "Position update: " << position_update.transpose() << std::endl;
            particle_i->positionUpdate(position_update);
        }

        // typename Constraint::ConstraintVecType newC = _constraint->evaluate();
        // std::cout << "New C: " << newC.transpose() << std::endl;
        // assert(0);
    }

    const typename Constraint::ConstraintVecType& lambda() const { return _lambda; }
    ConstVectorHandle<Constraint> constraint() const { return _constraint; }

private:
    typename Constraint::ConstraintVecType _lambda;
    ConstVectorHandle<Constraint> _constraint;
};

} // namespace Constraint