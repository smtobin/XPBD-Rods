#include "common/common.hpp"
#include "constraint/AllConstraints.hpp"

/** Tests each constraint and their gradients to ensure that the linearization is good.
 * 
 */


template<typename ConstraintType>
typename ConstraintType::GradientMatType numericalConstraintGradient(ConstraintType& constraint)
{
    const typename ConstraintType::ParticlePtrArray& particles = constraint.particles();
    typename ConstraintType::ConstraintVecType orig_C = constraint.evaluate();

    Real delta = 1e-8;
    typename ConstraintType::GradientMatType gradient;
    for (int pi = 0; pi < ConstraintType::NumParticles; pi++)
    {
        SimObject::OrientedParticle* particle_i = particles[pi];
        for (int dof_i = 0; dof_i < 3; dof_i++)
        {
            particle_i->position[dof_i] += delta;
            typename ConstraintType::ConstraintVecType new_C = constraint.evaluate();
            gradient.col(6*pi + dof_i) = (new_C - orig_C) / delta;
            particle_i->position[dof_i] -= delta;
        }
        for (int dof_i = 0; dof_i < 3; dof_i++)
        {
            Mat3r orig_or = particle_i->orientation;
            Vec3r dR = Vec3r::Zero();
            dR[dof_i] += delta;
            particle_i->orientation = Math::Plus_SO3(orig_or, dR);
            typename ConstraintType::ConstraintVecType new_C = constraint.evaluate();
            gradient.col(6*pi + 3 + dof_i) = (new_C - orig_C) / delta;
            particle_i->orientation = orig_or;
        }
    }

    return gradient;
}

template<typename ConstraintType>
bool testConstraint(ConstraintType& constraint)
{
    typename ConstraintType::GradientMatType gradient = constraint.gradient();
    typename ConstraintType::GradientMatType numerical_gradient = numericalConstraintGradient(constraint);

    std::cout << "\n === " << typeid(ConstraintType).name() << " === " << std::endl;
    Real max_diff = (gradient - numerical_gradient).cwiseAbs().maxCoeff();
    std::cout << "Max gradient diff: " << max_diff << std::endl;

    if (max_diff > 1e-5)
    {
        std::cout << " TEST FAILED! " << std::endl;
        std::cout << " Calculated gradient:\n" << gradient << std::endl;
        std::cout << "\n Numerical gradient:\n" << numerical_gradient << std::endl;
        return false;
    }

    return true;
}

SimObject::OrientedParticle randomParticle()
{
    Vec3r pos = Vec3r::Random();
    Vec3r R_vec = Vec3r::Random();
    SimObject::OrientedParticle particle;
    particle.position = pos;
    particle.orientation = Math::Exp_so3(R_vec);

    return particle;
}

Mat3r randomRotation()
{
    Vec3r R_vec = Vec3r::Random();
    return Math::Exp_so3(R_vec);
}


int main()
{
    randomRotation();
    SimObject::OrientedParticle particle1 = randomParticle();
    SimObject::OrientedParticle particle2 = randomParticle();

    Constraint::RevoluteJointConstraint                 rev_constraint1(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::OneSidedRevoluteJointConstraint         rev_constraint2(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    Constraint::NormedRevoluteJointConstraint           rev_constraint3(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::NormedOneSidedRevoluteJointConstraint   rev_constraint4(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    testConstraint(rev_constraint1);
    testConstraint(rev_constraint2);
    testConstraint(rev_constraint3);
    testConstraint(rev_constraint4);

    Constraint::SphericalJointConstraint                sph_constraint1(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::OneSidedSphericalJointConstraint        sph_constraint2(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    Constraint::NormedSphericalJointConstraint          sph_constraint3(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::NormedOneSidedSphericalJointConstraint  sph_constraint4(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    testConstraint(sph_constraint1);
    testConstraint(sph_constraint2);
    testConstraint(sph_constraint3);
    testConstraint(sph_constraint4);

    Constraint::PrismaticJointConstraint                pr_constraint1(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::OneSidedPrismaticJointConstraint        pr_constraint2(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    Constraint::NormedPrismaticJointConstraint          pr_constraint3(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::NormedOneSidedPrismaticJointConstraint  pr_constraint4(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    testConstraint(pr_constraint1);
    testConstraint(pr_constraint2);
    testConstraint(pr_constraint3);
    testConstraint(pr_constraint4);
    
}