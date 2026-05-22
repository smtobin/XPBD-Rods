#pragma once

#include "common/common.hpp"
#include "constraint/XPBDConstraintProjector_Base.hpp"
#include "simobject/OrientedParticle.hpp"

/** Helper struct to determine if the constraint class has evaluateSingle() */
template<typename, typename = void>
struct has_evaluate_single : std::false_type {};

template<typename T>
struct has_evaluate_single<
    T,
    std::void_t<
        decltype(std::declval<T>().evaluateSingle(std::declval<int>()))
    >
> : std::true_type {};

template<typename T>
constexpr bool has_evaluate_single_v = has_evaluate_single<T>::value;


namespace Constraint
{

template <typename Constraint_>
class XPBDSeparateConstraintProjector : public XPBDConstraintProjector_Base
{
public:
    using Constraint = Constraint_;

    XPBDSeparateConstraintProjector(Real dt, const ConstVectorHandle<Constraint>& constraint)
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
        // for (const auto& particle : _constraint->orientedParticles())
        for (int i = 0; i < Constraint::NumOrientedParticles; i++)
        {
            const SimObject::OrientedParticle* particle = _constraint->orientedParticles()[i];
            inertia_inverse.template block<6,1>(6*i, 0) = 
                Vec6r(1/particle->mass, 1/particle->mass, 1/particle->mass, 1/particle->Ib[0], 1/particle->Ib[1], 1/particle->Ib[2]);
        }
        // std::cout << "Inertia inverse: " << inertia_inverse.transpose()  << std::endl;

        // std::cout << "\n" << std::endl;
        for (int i = 0; i < Constraint::ConstraintDim; i++)
        {
            Real Ci;
            if constexpr(has_evaluate_single_v<Constraint>)
                Ci = _constraint->evaluateSingle(i);
            else
            {
                typename Constraint::ConstraintVecType C = _constraint->evaluate();
                Ci = C[i];

                assert(0);
            }
            

            // special handling for inequality constraints
            if (_constraint->isInequality())
            {
                // skip projecting this constraint if > 0 (i.e. constraint is not violated)
                if (Ci >= 0)
                    continue;
            }
            
            Eigen::Matrix<Real, 1, Constraint::StateDim> delCi;

            if constexpr(has_evaluate_single_v<Constraint>)
                delCi = _constraint->gradientSingle(i);
            else
            {
                typename Constraint::GradientMatType delC = _constraint->gradient();
                delCi = delC.row(i);
            }
            
            // std::cout << "delC:\n" << delC.transpose() << std::endl;

            Real RHS = -Ci - _constraint->alpha()[i] * _lambda[i] / (_dt*_dt);
            Real LHS = delCi * inertia_inverse.asDiagonal() * delCi.transpose() + _constraint->alpha()[i] / (_dt * _dt);

            // std::cout << "RHS: " << RHS << "  LHS: " << LHS << std::endl;

            Real dlam = RHS/LHS;
            _lambda[i] += dlam;

            // std::cout << "Dlam: " << dlam << std::endl;

            // update nodes
            for (int j = 0; j < Constraint::NumOrientedParticles; j++)
            {
                Vec6r delCi_particle_j = delCi.template block<1,6>(0,6*j);
                // std::cout << "Particle j grad: \n" << particle_j_grad << std::endl;
                // std::cout << "Inertia inverse block: " << inertia_inverse.template block<6,1>(6*j, 0).transpose() << std::endl;
                SimObject::OrientedParticle* particle_j = _constraint->orientedParticles()[j];
                const Vec6r position_update = inertia_inverse.template block<6,1>(6*j, 0).asDiagonal() * delCi_particle_j * dlam;
                // std::cout << "Position update: " << position_update.transpose() << std::endl;
                particle_j->positionUpdate(position_update);
            }
            // typename Constraint::ConstraintVecType newC = _constraint->evaluate();
            // std::cout << "New C: " << newC.transpose() << std::endl;
        }

        

        
        // assert(0);
    }
    
    const typename Constraint::ConstraintVecType& lambda() const { return _lambda; }
    ConstVectorHandle<Constraint> constraint() const { return _constraint; }

private:
    typename Constraint::ConstraintVecType _lambda;
    ConstVectorHandle<Constraint> _constraint;
};

} // namespace Constraint