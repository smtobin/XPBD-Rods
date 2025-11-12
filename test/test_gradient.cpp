#include "constraint/RodElasticConstraint.hpp"
#include "constraint/RevoluteJointConstraint.hpp"

#include "simobject/OrientedParticle.hpp"

int main()
{
    // test RodElasticConstraint
    {
        SimObject::OrientedParticle p1, p2;
        p1.position = Vec3r(0,0,0);
        p1.orientation = Mat3r::Identity();
        p2.position = Vec3r(0.5,0.2,0.3);
        p2.orientation = Math::Exp_so3(Vec3r(0.4, 0.5, 0.3));

        Constraint::RodElasticConstraint constraint(&p1, &p2, Vec6r::Zero()); 

        auto C1 = constraint.evaluate();
        auto grad1 = constraint.gradient();

        Vec3r dp1(0.002, -0.001, 0.004);
        Vec3r dR1(0.002, -0.004, 0.005);
        Vec3r dp2(0.008, 0.002, 0.006);
        Vec3r dR2(0.003, 0.004, -0.004);
        Eigen::Vector<Real, 12> dx;
        dx << dp1, dR1, dp2, dR2;

        auto predicted_C2 = C1 + grad1 * dx;

        p1.positionUpdate(dp1, dR1);
        p2.positionUpdate(dp2, dR2);

        auto C2 = constraint.evaluate();

        std::cout << "\n=== RodElasticConstraint ===" << std::endl;
        std::cout << "C1: " << C1.transpose() << "\nC2 actual:\t" << C2.transpose() << "\nC2 predicted:\t" << predicted_C2.transpose() << std::endl;
    }

    // test NormedOneSidedRevoluteJointConstraint
    {
        SimObject::OrientedParticle p1, p2;
        p1.position = Vec3r(0,0,0);
        p1.orientation = Mat3r::Identity();
        p2.position = Vec3r(0.5,0.2,0.3);
        p2.orientation = Math::Exp_so3(Vec3r(0.4, 0.5, 0.3));

        Constraint::NormedOneSidedRevoluteJointConstraint constraint(Vec3r(0,0,0), Math::Exp_so3(Vec3r(-0.2,-0.2,0.4)), &p2, Vec3r(0,0.1,0), Math::Exp_so3(Vec3r(0.1,0.2,0.3))); 

        auto C1 = constraint.evaluate();
        auto grad1 = constraint.gradient();

        std::cout << "grad1: " << grad1 << std::endl;

        Vec3r dp1(0.002, -0.001, 0.004);
        Vec3r dR1(0.002, -0.004, 0.005);
        Vec3r dp2(0.008, 0.002, 0.006);
        Vec3r dR2(0.003, 0.004, -0.004);
        Eigen::Vector<Real, 6> dx;
        dx << dp2, dR2;

        auto predicted_C2 = C1 + grad1 * dx;

        p2.positionUpdate(dp2, dR2);

        auto C2 = constraint.evaluate();

        std::cout << "\n=== NormedOneSidedRevoluteJointConstraint ===" << std::endl;
        std::cout << "C1: " << C1.transpose() << "\nC2 actual:\t" << C2.transpose() << "\nC2 predicted:\t" << predicted_C2.transpose() << std::endl;
    }
    

}