#pragma once

#include "common/common.hpp"
#include "constraint/FixedJointConstraint.hpp"
#include "constraint/RevoluteJointConstraint.hpp"


struct Muller2020ConstraintHelper
{
    template<typename Constraint>
    static Vec3r positionalCorrection(const Constraint* constraint)
    {
        Vec3r dp = constraint->evaluate().template head<3>();
        return dp;
    }

    template<typename Constraint>
    static Vec3r angularCorrection(const Constraint* constraint)
    {
        Vec3r dq = constraint->evaluate().template tail<3>();
        return dq;
    }

    template<typename Constraint>
    static Real positionalW1(const Constraint* constraint)
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
    static Real positionalW2(const Constraint* constraint)
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
    static Real angularW1(const Constraint* constraint)
    {
        const SimObject::OrientedParticle* particle1 = constraint->particles()[0];
        Vec3r dor = angularCorrection(constraint);
        Vec3r n = dor / dor.norm();

        Vec3r I_inv = 1/particle1->Ib.array();
        return n.transpose() * particle1->orientation.transpose() * I_inv.asDiagonal() * particle1->orientation * n;
    }

    template<typename Constraint>
    static Real angularW2(const Constraint* constraint)
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
            return n.transpose() * particle2->orientation.transpose() * I_inv.asDiagonal() * particle2->orientation * n;
        }
    }

    /** RevoluteJointConstraint specialization */
    static Vec3r angularCorrection(const Constraint::RevoluteJointConstraint* constraint)
    {
        Vec3r a1 = constraint->jointOrientation1().col(2);
        Vec3r a2 = constraint->jointOrientation2().col(2);

        return a1.cross(a2);
    }

    static Vec3r angularCorrection(const Constraint::OneSidedRevoluteJointConstraint* constraint)
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
    Muller2020ConstraintProjector(Real dt, const Constraint* constraint)
        : _dt(dt), _lambda(0), _constraint(constraint)
    {

    }

    void initialize()
    {
        _lambda = 0;
    }

    void project()
    {
        _projectPosition();
        _projectOrientation();
    }

    // projects the "positional" part of the constraint
    // corresponds to Sec 3.3.1 "Positional Constraints" in rigid-body paper.
    void _projectPosition()
    {
        // difference in positions of joint axes on each body
        Vec3r dp = Muller2020ConstraintHelper::positionalCorrection(_constraint);
        // constraint violation and direction
        Real c = dp.norm();
        Vec3r n = dp / c;

        // get denominator weights from helper
        Real w1 = Muller2020ConstraintHelper::positionalW1(_constraint);
        Real w2 = Muller2020ConstraintHelper::positionalW2(_constraint);

        // XPBD formula (assuming alpha = 0 for now)
        Real dlam = -c / (w1 + w2); // <== assuming alpha = 0 for now

        // update lambda
        _lambda += dlam;

        // compute position and orientation update for each particle

        // particle 1 position
        SimObject::OrientedParticle* particle1 = _constraint->positions()[0];
        Vec3r p1_update = n * dlam / _constraint->positions()[0]->mass;
        // particle 1 orientation
        Vec3r r1 = _constraint->bodyJointOffset1();
        Vec3r p1_Ib_inv = 1/particle1->Ib.array();
        Vec3r or1_update = p1_Ib_inv.asDiagonal() * (r1.cross(dlam*n));
        particle1->positionUpdate(p1_update, or1_update);

        // particle 2 (if applicable)
        if (Constraint::NumParticles == 2)
        {
            SimObject::OrientedParticle* particle2 = _constraint->positions()[1];
            Vec3r p2_update = dlam * n / particle2->mass;

            Vec3r r2 = _constraint->bodyJointOffset2();
            Vec3r p2_Ib_inv = 1/particle2->Ib.array();
            Vec3r or2_update = p2_Ib_inv.asDiagonal() * (-r2.cross(dlam*n));
            particle2->positionUpdate(p2_update, or2_update);
        }

    }

    // projects the "angular" part of the constraint
    // corresponds to Sec 3.3.2 of the rigid-body paper
    void _projectOrientation()
    {
        // difference in orientations of joint axes on each body
        Vec3r dor = Muller2020ConstraintHelper::angularCorrection(_constraint);
        // constraint violation and direction
        Real theta = dor.norm();
        Vec3r n = dor / theta;

        // get denominator weights from helper
        Real w1 = Muller2020ConstraintHelper::angularW1(_constraint);
        Real w2 = Muller2020ConstraintHelper::angularW2(_constraint);

        // XPBD formula (assuming alpha = 0 for now)
        Real dlam = -theta / (w1 + w2); // <== assuming alpha = 0 for now

        // update lambda
        _lambda += dlam;

        // compute orientation update for each particle
        SimObject::OrientedParticle* particle1 = _constraint->positions()[0];
        Vec3r p1_Ib_inv = 1/particle1->Ib.array();
        Vec3r or1_update = p1_Ib_inv.asDiagonal() * (dlam * n);
        particle1->positionUpdate(Vec3r::Zero(), or1_update);

        if (Constraint::NumParticles == 2)
        {
            SimObject::OrientedParticle* particle2 = _constraint->positions()[1];
            Vec3r p2_Ib_inv = 1/particle2->Ib.array();
            Vec3r or2_update = p2_Ib_inv.asDiagonal() * (-dlam * n);
            particle2->positionUpdate(Vec3r::Zero(), or2_update);
        }

    }



private:
    Real _dt;
    Real _lambda;
    const Constraint* _constraint;

};