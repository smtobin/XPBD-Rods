#pragma once

#include "common/common.hpp"
#include "constraint/XPBDConstraintProjector_Base.hpp"
#include "simobject/OrientedParticle.hpp"

namespace Constraint
{

template <typename Constraint_>
class XPBDSeparateConstraintProjector : public XPBDConstraintProjector_Base
{
public:
    using Constraint = Constraint_;

    XPBDSeparateConstraintProjector(Real dt, const Constraint* constraint)
        : XPBDConstraintProjector_Base(dt), _constraint(constraint)
    {
    }

    virtual void initialize() override
    {
        _lambda = Constraint::ConstraintVecType::Zero();
    }

    virtual void project() override
    {
        Eigen::Vector<Real, Constraint::StateDim> inertia_inverse;
        // for (const auto& particle : _constraint->particles())
        for (int i = 0; i < Constraint::NumParticles; i++)
        {
            const SimObject::OrientedParticle* particle = _constraint->particles()[i];
            inertia_inverse.template block<6,1>(6*i, 0) = 
                Vec6r(1/particle->mass, 1/particle->mass, 1/particle->mass, 1/particle->Ib[0], 1/particle->Ib[1], 1/particle->Ib[2]);
        }

        // std::cout << "\n" << std::endl;
        for (int i = 0; i < Constraint::ConstraintDim; i++)
        {
            typename Constraint::ConstraintVecType C = _constraint->evaluate();
            // std::cout << "========\nC: " << C.transpose() << std::endl;
            typename Constraint::GradientMatType delC = _constraint->gradient(true);
            // std::cout << "delC:\n" << delC.transpose() << std::endl;

            Real RHS = -C[i] - _constraint->alpha()[i] * _lambda[i] / (_dt*_dt);
            Real LHS = delC.row(i) * inertia_inverse.asDiagonal() * delC.row(i).transpose() + _constraint->alpha()[i] / (_dt * _dt);

            // std::cout << "RHS: " << RHS << "  LHS: " << LHS << std::endl;

            Real dlam = RHS/LHS;
            _lambda[i] += dlam;

            // update nodes
            for (int j = 0; j < Constraint::NumParticles; j++)
            {
                SimObject::OrientedParticle* particle_j = _constraint->particles()[j];
                // std::cout << "Single particle gradient:\n" << _constraint->singleParticleGradient(particle_i, true).transpose() << std::endl;
                typename Constraint::SingleParticleGradientMatType grad_j = _constraint->singleParticleGradient(particle_j, true);
                // std::cout << "grad_j.row(i).transpose(): " << grad_j.row(i).transpose() << std::endl;
                const Vec6r position_update = inertia_inverse.template block<6,1>(6*j, 0).asDiagonal() * grad_j.row(i).transpose() * dlam;
                // std::cout << "Position update: " << position_update.transpose() << std::endl;
                particle_j->positionUpdate(position_update);
            }
            // typename Constraint::ConstraintVecType newC = _constraint->evaluate();
            // std::cout << "New C: " << newC.transpose() << std::endl;
        }

        

        
        // assert(0);
    }
    
    const typename Constraint::ConstraintVecType& lambda() const { return _lambda; }
    const Constraint* constraint() const { return _constraint; }

private:
    typename Constraint::ConstraintVecType _lambda;
    const Constraint* _constraint;
};

} // namespace Constraint