#pragma once

#include "common/common.hpp"
#include "constraint/FixedJointConstraint.hpp"
#include "constraint/RevoluteJointConstraint.hpp"

#include <Eigen/Geometry>

namespace Constraint
{
struct Muller2020ConstraintHelper
{
    template<typename Constraint>
    static Vec3r positionalCorrection(const ConstVectorHandle<Constraint>& constraint)
    {
        Vec3r dp = constraint->evaluate().template head<3>();
        return dp;
    }

    template<typename Constraint>
    static Vec3r angularCorrection(const ConstVectorHandle<Constraint>& constraint)
    {
        std::cout << "\n\nWRONG ANGULAR CORRECTION!" << std::endl;
        Vec3r dq = constraint->evaluate().template tail<3>();
        return dq;
    }

    template<typename Constraint>
    static Real positionalW1(const ConstVectorHandle<Constraint>& constraint)
    {   
        const SimObject::OrientedParticle* particle1 = constraint->particles()[0];
        Vec3r dp = positionalCorrection(constraint);
        Vec3r dp_local = particle1->orientation.transpose() * dp;    // put dp from global to local frame
        Vec3r n = dp_local/dp_local.norm();

        Vec3r r1 = constraint->bodyJointOffset1();
        Vec3r r1_x_n = r1.cross(n);
        Vec3r I_inv = 1/particle1->Ib.array();
        return 1/particle1->mass + r1_x_n.transpose() * I_inv.asDiagonal() * r1_x_n;
    }

    template<typename Constraint>
    static Real positionalW2(const ConstVectorHandle<Constraint>& constraint)
    {
        if constexpr (Constraint::NumParticles == 1)
        {
            return 0;
        }
        else
        {
            const SimObject::OrientedParticle* particle2 = constraint->particles()[1];
            Vec3r dp = positionalCorrection(constraint);
            Vec3r dp_local = particle2->orientation.transpose() * dp;    // put dp from global to local frame
            Vec3r n = dp_local/dp_local.norm();

            Vec3r r2 = constraint->bodyJointOffset2();
            Vec3r r2_x_n = r2.cross(n);
            Vec3r I_inv = 1/particle2->Ib.array();
            return 1/particle2->mass + r2_x_n.transpose() * I_inv.asDiagonal() * r2_x_n;
        }
    }

    template<typename Constraint>
    static Real angularW1(const ConstVectorHandle<Constraint>& constraint)
    {
        const SimObject::OrientedParticle* particle1 = constraint->particles()[0];
        Vec3r dor = angularCorrection(constraint);
        Vec3r n = dor / dor.norm();

        Vec3r I_inv = 1/particle1->Ib.array();
        return n.transpose() * particle1->orientation * I_inv.asDiagonal() * particle1->orientation.transpose() * n;
    }

    template<typename Constraint>
    static Real angularW2(const ConstVectorHandle<Constraint>& constraint)
    {
        if constexpr (Constraint::NumParticles == 1)
        {
            return 0;
        }
        else
        {
            const SimObject::OrientedParticle* particle2 = constraint->particles()[1];
            Vec3r dor = angularCorrection(constraint);
            Vec3r n = dor / dor.norm();

            Vec3r I_inv = 1/particle2->Ib.array();
            return n.transpose() * particle2->orientation * I_inv.asDiagonal() * particle2->orientation.transpose() * n;
        }
    }

    /** RevoluteJointConstraint specialization */
    static Vec3r angularCorrection(const ConstVectorHandle<Constraint::RevoluteJointConstraint>& constraint)
    {
        Vec3r a1 = constraint->jointOrientation1().col(2);
        Vec3r a2 = constraint->jointOrientation2().col(2);

        return a1.cross(a2);
    }

    static Vec3r angularCorrection(const ConstVectorHandle<Constraint::OneSidedRevoluteJointConstraint>& constraint)
    {
        Vec3r a1 = constraint->jointOrientation1().col(2);  // joint orientation 1 is fixed for a one-sided constraint
        Vec3r a2 = constraint->jointOrientation2().col(2);

        return a1.cross(a2);
    }

};

template <typename Constraint>
class Muller2020ConstraintProjector
{
public:
    Muller2020ConstraintProjector(Real dt, ConstVectorHandle<Constraint> constraint)
        : _dt(dt), _positional_lambda(0), _angular_lambda(0), _constraint(constraint)
    {

    }

    void initialize()
    {
        _positional_lambda = 0;
        _angular_lambda = 0;
    }

    void project()
    {
        if constexpr (TypeListContains<Constraint, XPBDRigidBodyConstraints_TypeList>::value)
        {
            Vec3r dp = Muller2020ConstraintHelper::positionalCorrection(_constraint);
            Vec3r dor = Muller2020ConstraintHelper::angularCorrection(_constraint);
            std::cout << "\n================\nC: " << dp.norm() << ", " << dor.norm() << std::endl; 
            _projectPosition();
            dp = Muller2020ConstraintHelper::positionalCorrection(_constraint);
            dor = Muller2020ConstraintHelper::angularCorrection(_constraint);
            std::cout << "C: " << dp.norm() << ", " << dor.norm() << std::endl; 
            _projectOrientation();
            dp = Muller2020ConstraintHelper::positionalCorrection(_constraint);
            dor = Muller2020ConstraintHelper::angularCorrection(_constraint);
            std::cout << "Final C: " << dp.norm() << ", " << dor.norm() << std::endl; 
        }
        else
        {
            // can't project non-rigid-body constraint types
            assert(0);
        }
        
    }

    // projects the "positional" part of the constraint
    // corresponds to Sec 3.3.1 "Positional Constraints" in rigid-body paper.
    void _projectPosition()
    {
        // difference in positions of joint axes on each body
        Vec3r dp = Muller2020ConstraintHelper::positionalCorrection(_constraint);
        // constraint violation and direction
        Real c = dp.norm();

        if (c < 1e-12)
            return;

        Vec3r n = dp / c;

        // get denominator weights from helper
        Real w1 = Muller2020ConstraintHelper::positionalW1(_constraint);
        Real w2 = Muller2020ConstraintHelper::positionalW2(_constraint);

        // XPBD formula (assuming alpha = 0 for now)
        Real dlam = -c / (w1 + w2); // <== assuming alpha = 0 for now

        // update lambda
        _positional_lambda += dlam;

        // compute position and orientation update for each particle

        // particle 1 position
        SimObject::OrientedParticle* particle1 = _constraint->particles()[0];
        Vec3r p1_update = -n * dlam / particle1->mass;
        // particle 1 orientation
        Vec3r r1 = _constraint->bodyJointOffset1();
        Vec3r p1_Ib_inv = 1/particle1->Ib.array();
        Vec3r or1_update = p1_Ib_inv.asDiagonal() * (r1.cross(-dlam*particle1->orientation.transpose()*n));
        // std::cout << "Position update: " << p1_update.transpose() << "\tOrientation update: " << or1_update.transpose() << std::endl;
        particle1->positionUpdate(p1_update, or1_update);

        // particle 2 (if applicable)
        if (Constraint::NumParticles == 2)
        {
            SimObject::OrientedParticle* particle2 = _constraint->particles()[1];
            Vec3r p2_update = dlam * n / particle2->mass;

            Vec3r r2 = _constraint->bodyJointOffset2();
            Vec3r p2_Ib_inv = 1/particle2->Ib.array();
            Vec3r or2_update = p2_Ib_inv.asDiagonal() * (r2.cross(dlam*particle2->orientation.transpose()*n));
            particle2->positionUpdate(p2_update, or2_update);
        }

    }

    // projects the "angular" part of the constraint
    // corresponds to Sec 3.3.2 of the rigid-body paper
    void _projectOrientation()
    {
        // difference in orientations of joint axes on each body
        Vec3r dor = Muller2020ConstraintHelper::angularCorrection(_constraint);
        // std::cout << "\nAngular correction: "<< dor.transpose() << std::endl;
        // constraint violation and direction
        Real theta = dor.norm();

        if (theta < 1e-12)
            return;

        Vec3r n = dor / theta;

        // std::cout << "n local p1: " << (_constraint->particles()[0]->orientation.transpose() * n).transpose() << std::endl;

        // get denominator weights from helper
        Real w1 = Muller2020ConstraintHelper::angularW1(_constraint);
        Real w2 = Muller2020ConstraintHelper::angularW2(_constraint);

        std::cout << "RHS: " << -theta << "  LHS: " << w1+w2 << std::endl;
        // XPBD formula (assuming alpha = 0 for now)
        Real dlam = -theta / (w1 + w2); // <== assuming alpha = 0 for now

        // update lambda
        _angular_lambda += dlam;

        // compute orientation update for each particle
        SimObject::OrientedParticle* particle1 = _constraint->particles()[0];
        Vec3r p1_Ib_inv = 1/particle1->Ib.array();
        Vec3r or1_update = p1_Ib_inv.asDiagonal() * (-dlam * particle1->orientation.transpose() * n);
        std::cout << "p1 Orientation update: " << or1_update.transpose() << std::endl;
        particle1->positionUpdate(Vec3r::Zero(), or1_update);

        if (Constraint::NumParticles == 2)
        {
            SimObject::OrientedParticle* particle2 = _constraint->particles()[1];
            Vec3r p2_Ib_inv = 1/particle2->Ib.array();
            Vec3r or2_update = p2_Ib_inv.asDiagonal() * (dlam * particle2->orientation.transpose() * n);
            // std::cout << "p2 Orientation update: " << or2_update.transpose() << std::endl;
            particle2->positionUpdate(Vec3r::Zero(), or2_update);
        }

        // std::cout << "New angular correction: " << Muller2020ConstraintHelper::angularCorrection(_constraint).transpose() << std::endl;

    }



private:
    Real _dt;
    Real _positional_lambda;
    Real _angular_lambda;
    ConstVectorHandle<Constraint> _constraint;

};

} // namespace Constraint