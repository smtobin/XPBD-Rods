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
    SimObject::OrientedParticle particle3 = randomParticle();

    /** Revolute joint constraints */
    Constraint::RevoluteJointConstraint                 rev_constraint1(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::OneSidedRevoluteJointConstraint         rev_constraint2(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    Constraint::NormedRevoluteJointConstraint           rev_constraint3(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::NormedOneSidedRevoluteJointConstraint   rev_constraint4(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    testConstraint(rev_constraint1);
    testConstraint(rev_constraint2);
    testConstraint(rev_constraint3);
    testConstraint(rev_constraint4);

    /** Spherical joint constraints */
    Constraint::SphericalJointConstraint                sph_constraint1(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::OneSidedSphericalJointConstraint        sph_constraint2(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    Constraint::NormedSphericalJointConstraint          sph_constraint3(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::NormedOneSidedSphericalJointConstraint  sph_constraint4(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    testConstraint(sph_constraint1);
    testConstraint(sph_constraint2);
    testConstraint(sph_constraint3);
    testConstraint(sph_constraint4);

    /** Prismatic joint constraints */
    Constraint::PrismaticJointConstraint                pr_constraint1(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::OneSidedPrismaticJointConstraint        pr_constraint2(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    Constraint::NormedPrismaticJointConstraint          pr_constraint3(&particle1, Vec3r::Random(), randomRotation(), &particle2, Vec3r::Random(), randomRotation());
    Constraint::NormedOneSidedPrismaticJointConstraint  pr_constraint4(Vec3r::Random(), randomRotation(), &particle1, Vec3r::Random(), randomRotation());
    testConstraint(pr_constraint1);
    testConstraint(pr_constraint2);
    testConstraint(pr_constraint3);
    testConstraint(pr_constraint4);

    /** Joint limit constraints */
    Constraint::RevoluteJointLimitConstraint            rev_lim_constraint1(rev_constraint1, -0.5, 0.5);
    Constraint::OneSidedRevoluteJointLimitConstraint    rev_lim_constraint2(rev_constraint2, -0.5, 0.5);
    testConstraint(rev_lim_constraint1);
    testConstraint(rev_lim_constraint2);

    Constraint::SphericalJointSwingLimitConstraint          sph_swing_lim_constraint1(sph_constraint1, 0.1, 0.8);
    Constraint::OneSidedSphericalJointSwingLimitConstraint  sph_swing_lim_constraint2(sph_constraint2, 0.1, 0.8);
    testConstraint(sph_swing_lim_constraint1);
    testConstraint(sph_swing_lim_constraint2);

    Constraint::PrismaticJointLimitConstraint           pr_lim_constraint1(pr_constraint1, -0.5, 0.5);
    Constraint::OneSidedPrismaticJointLimitConstraint   pr_lim_constraint2(pr_constraint2, -0.5, 0.5);
    testConstraint(pr_lim_constraint1);
    testConstraint(pr_lim_constraint2);

    /** Rod Elastic Gauss Constraints */
    std::array<SimObject::OrientedParticle*, 2> o1_element_particles = {&particle1, &particle2};
    SimObject::RodElement<1> o1_element(o1_element_particles, 0.5);
    Constraint::RodElasticGaussPointConstraint<1> o1_constraint(&o1_element, 0.5, Vec6r::Zero());
    testConstraint(o1_constraint);

    std::array<SimObject::OrientedParticle*, 3> o2_element_particles = {&particle1, &particle2, &particle3};
    SimObject::RodElement<2> o2_element(o2_element_particles, 0.5);
    Constraint::RodElasticGaussPointConstraint<2> o2_constraint(&o2_element, 0.33, Vec6r::Zero());
    testConstraint(o2_constraint);

    /** Test derivative of exponential map */
    Real l = 0.1;
    Vec3r theta1(0.5, -0.6, 0.4);
    Vec3r theta2(0.8, 0.2, 0.9);
    Mat3r R1 = Math::Exp_so3(theta1);
    Mat3r R2 = Math::Exp_so3(theta2);
    Vec3r theta = Math::Minus_SO3(R2, R1);
    Real s = 0.5;
    Vec3r theta_prime = 1/l * theta;


    Mat3r gamma = Math::ExpMap_RightJacobian(s*theta);
    Mat3r gamma_inv = Math::ExpMap_InvRightJacobian(theta);

    Vec3r u = gamma * theta_prime;

    Vec3r dR1(0.001, -0.002, -0.001);
    Vec3r u_pred_delta1 = Math::DExpMap_RightJacobian_Contract_k(s*theta, -s*gamma_inv.transpose() * dR1) * theta_prime + gamma * -1/l * gamma_inv.transpose() * dR1;
    Vec3r u_pred_delta2 = (Math::DExpMap_RightJacobian_Contract_j(s*theta, theta_prime) * -s*gamma_inv.transpose() + gamma * -1/l * gamma_inv.transpose()) * dR1;
    Vec3r u_pred_delta3 = -1/l * gamma_inv.transpose() * dR1;


    R1 = Math::Plus_SO3(R1, dR1);
    Vec3r u_new = Math::ExpMap_RightJacobian(s*Math::Minus_SO3(R2, R1)) * 1/l * Math::Minus_SO3(R2, R1);

    std::cout << "Original u:\n" << u.transpose() << std::endl;
    std::cout << "Predicted du1:\n" << u_pred_delta1.transpose() << std::endl;
    std::cout << "Predicted du2:\n" << u_pred_delta2.transpose() << std::endl;
    std::cout << "Predicted du3:\n" << u_pred_delta3.transpose() << std::endl;
    std::cout << "Actual du:\n" << (u_new - u).transpose() << std::endl;

    Vec3r v1(1.2, 0.8, 0.6);
    Vec3r v2(0.7, 0.2, 0.4);
    Mat3r A;
    A << 0.5, 0.9, -0.4, 0.6, 0.5, 0.3, 0.1, -0.1, 0.6;
    Vec3r res1 = Math::DExpMap_RightJacobian_Contract_k(s*theta, A*v1) * theta_prime;
    Vec3r res2 = Math::DExpMap_RightJacobian_Contract_j(s*theta, theta_prime) * A * v1;

    std::cout << "res1: " << res1.transpose() << std::endl;
    std::cout << "res2: " << res2.transpose() << std::endl;
    
}