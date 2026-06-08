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
        // std::cout << "========\nC: " << C.transpose() << std::endl;

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
        
        typename Constraint::GradientMatType delC = _constraint->gradient();
        // std::cout << "delC:\n" << delC.transpose() << std::endl;

        const typename Constraint::ConstraintVecType RHS = -C - _constraint->alpha().asDiagonal() * _lambda / (_dt*_dt);

        Eigen::Vector<Real, Constraint::StateDim> inertia_inverse;
        // for (const auto& particle : _constraint->orientedParticles())
        for (int i = 0; i < Constraint::NumOrientedParticles; i++)
        {
            const SimObject::OrientedParticle* particle = _constraint->orientedParticles()[i];
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
        for (int i = 0; i < Constraint::NumOrientedParticles; i++)
        {
            using SingleOrientedParticleGradientMatType = Eigen::Matrix<Real, Constraint::ConstraintDim, 6>;
            SingleOrientedParticleGradientMatType particle_i_grad = delC.template block<Constraint::ConstraintDim, 6>(0, 6*i);
            SimObject::OrientedParticle* particle_i = _constraint->orientedParticles()[i];
            // std::cout << "Single particle gradient:\n" << _constraint->singleParticleGradient(particle_i, true).transpose() << std::endl;
            const Vec6r position_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dlam;
            // std::cout << "Position update: " << position_update.transpose() << std::endl;
            particle_i->positionUpdate(position_update);
        }

        // typename Constraint::ConstraintVecType newC = _constraint->evaluate();
        // std::cout << "New C: " << newC.transpose() << std::endl;
        // assert(0);
    }

    virtual void projectVelocity() override
    {
        typename Constraint::GradientMatType delC = _constraint->gradient();

        Eigen::Vector<Real, Constraint::StateDim> inertia_inverse;
        Eigen::Vector<Real, Constraint::StateDim> velocity_vec;

        // for (const auto& particle : _constraint->orientedParticles())
        for (int i = 0; i < Constraint::NumOrientedParticles; i++)
        {
            const SimObject::OrientedParticle* particle = _constraint->orientedParticles()[i];
            inertia_inverse.template block<6,1>(6*i, 0) = 
                Vec6r(1/particle->mass, 1/particle->mass, 1/particle->mass, 1/particle->Ib[0], 1/particle->Ib[1], 1/particle->Ib[2]);
            velocity_vec.template block<3,1>(6*i, 0) = particle->lin_velocity;
            velocity_vec.template block<3,1>(6*i+3, 0) = particle->ang_velocity;
        }

        /** TODO: check this */
        const typename Constraint::ConstraintVecType RHS = -_constraint->beta() * _dt * _constraint->alpha().asDiagonal() * delC * velocity_vec;       

        const Eigen::Matrix<Real, Constraint::ConstraintDim, Constraint::ConstraintDim> LHS =
            delC * inertia_inverse.asDiagonal() * delC.transpose();

        const typename Constraint::ConstraintVecType dmu = LHS.llt().solve(RHS);

        // update nodes
        for (int i = 0; i < Constraint::NumOrientedParticles; i++)
        {
            using SingleOrientedParticleGradientMatType = Eigen::Matrix<Real, Constraint::ConstraintDim, 6>;
            SingleOrientedParticleGradientMatType particle_i_grad = delC.template block<Constraint::ConstraintDim, 6>(0, 6*i);
            SimObject::OrientedParticle* particle_i = _constraint->orientedParticles()[i];
            // std::cout << "Single particle gradient:\n" << _constraint->singleParticleGradient(particle_i, true).transpose() << std::endl;
            const Vec6r velocity_update = inertia_inverse.template block<6,1>(6*i, 0).asDiagonal() * particle_i_grad.transpose() * dmu;
            // std::cout << "Position update: " << position_update.transpose() << std::endl;
            particle_i->lin_velocity += velocity_update.head<3>();
            particle_i->ang_velocity += velocity_update.tail<3>();
        }

    }

    const typename Constraint::ConstraintVecType& lambda() const { return _lambda; }
    ConstVectorHandle<Constraint> constraint() const { return _constraint; }

private:
    /** Positional Lagrange multipliers */
    typename Constraint::ConstraintVecType _lambda;

    /** The constraint we are projecting */
    ConstVectorHandle<Constraint> _constraint;
};

} // namespace Constraint