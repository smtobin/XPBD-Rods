#pragma once

#include "simobject/rod/RodElement.hpp"
#include "collision/sdf/SDF.hpp"

namespace Collision
{

class RodElementCollider
{
public:

static Real closestPointBetweenRodElementAndSDF(const SimObject::RodElement_Base* elem, const SDF* sdf)
{
    // fixed number of initial points to test
    const Real samples[7] = {
        0.0, 0.1666667, 0.3333333,
        0.5,
        0.6666667, 0.8333333, 1.0
    };

    Real best_s = 0.0;
    Real best_f = sdf->evaluate(elem->position(0.0));

    for (int i = 0; i < 7; i++)
    {
        double s = samples[i];
        double f = sdf->evaluate(elem->position(s));

        if (f < best_f)
        {
            best_f = f;
            best_s = s;
        }
    }

    std::cout << "Initial best s: " << best_s << "\tInitial best f: " << best_f << std::endl;

    // mini golden-section search
    Real a = std::max(Real(0.0), best_s - 0.2);
    Real b = std::min(Real(1.0), best_s + 0.2);
    const Real phi = 0.61803398875;
    Real x = best_s;
    Real fx = best_f;
    for (int i = 0; i < 4; i++)
    {
        Real left = x - phi * (x - a);
        Real right = x + phi * (b - x);

        Real fl = sdf->evaluate(elem->position(left));
        Real fr = sdf->evaluate(elem->position(right));

        if (fl < fx)
        {
            b = x;
            x = left;
            fx = fl;
        }
        else if (fr < fx)
        {
            a = x;
            x = right;
            fx = fr;
        }
        else
        {
            a = left;
            b = right;
        }
    }

    return x;
    
}

/** Finds closest points between two rod elements. Uses damped Gauss-Newton method to solve the optimization problem that minimizes squared error. */
static std::vector<std::pair<Real, Real>> closestPointsBetweenRodElements(const SimObject::RodElement_Base* elem1, const SimObject::RodElement_Base* elem2)
{
    // get initial estimates based on linear approximation
    int order1 = elem1->order();
    int order2 = elem2->order();
    std::vector<std::pair<Real, Real>> seeds(order1*order2);
    for (int i = 0; i < order1; i++)
    {
        for (int j = 0; j < order2; j++)
        {
            auto [s1_loc, s2_loc] = Math::findClosestPointsOnLineSegments(
                elem1->node(i)->position, elem1->node(i+1)->position, elem2->node(j)->position, elem2->node(j+1)->position
            );
            Real s1 = s1_loc / order1 + Real(i) / order1;
            Real s2 = s2_loc / order2 + Real(j) / order2;

            seeds[i*order1 + j] = std::make_pair(s1, s2);

            std::cout << "starting s1 and s2: " << s1 << ", " << s2 << std::endl;
        }
    }

    for (unsigned i = 0; i < seeds.size(); i++)
    {
        Real s1 = seeds[i].first;
        Real s2 = seeds[i].second;

        int num_iters = 5;
        // Newton's method
        // for (int iter = 0; iter < num_iters; iter++)
        // {
        //     Vec3r p1 = elem1->position(s1);
        //     Vec3r dp1 = elem1->dposition_dshat(s1);
        //     Vec3r d2p1 = elem1->d2position_dshat2(s1);
        //     Vec3r p2 = elem2->position(s2);
        //     Vec3r dp2 = elem2->dposition_dshat(s2);
        //     Vec3r d2p2 = elem2->d2position_dshat2(s2);

        //     Vec3r diff = p1 - p2;

        //     Mat2r J;
        //     J(0,0) = 2*dp1.dot(dp1) + 2*diff.dot(d2p1);
        //     J(0,1) = -2*dp1.dot(dp2);
        //     J(1,0) = J(0,1);
        //     J(1,1) = 2*dp2.dot(dp2) - 2*diff.dot(d2p2);

        //     Vec2r res;
        //     res[0] = -2*diff.dot(dp1);
        //     res[1] = 2*diff.dot(dp2);

        //     Real det = J(0,0)*J(1,1) - J(0,1)*J(1,0);

        //     if (det < 1e-12)
        //         break;
            
        //     Vec2r ds = Vec2r(
        //         J(1,1)*res(0) - J(0,1)*res(1),
        //         -J(1,0)*res(0) + J(0,0)*res(1)
        //     ) / det;

        //     std::cout << "ds: " << ds.transpose() << std::endl;

        //     s1 = std::clamp(s1 + ds[0], 0.0, 1.0);
        //     s2 = std::clamp(s2 + ds[1], 0.0, 1.0);
        // }

        /** Gauss-Newton
         * 
         * f(x) = ||r(x)||^2
         * 
         * (J^T * J) dx = -J^T * r
         * 
         * where J is the jacobian of r, the residual vector (p1 - p2)
         */
        Real damping = 0.5;
        for (int iter = 0; iter < num_iters; iter++)
        {
            Vec3r p1 = elem1->position(s1);
            Vec3r dp1 = elem1->dposition_dshat(s1);
            Vec3r p2 = elem2->position(s2);
            Vec3r dp2 = elem2->dposition_dshat(s2);
            Eigen::Matrix<Real, 3, 2> J;
            J.col(0) = dp1;
            J.col(1) = -dp2;

            Vec2r res = -J.transpose() * (p1 - p2);


            Mat2r JTJ = J.transpose()*J;
            Real det = JTJ(0,0)*JTJ(1,1) - JTJ(0,1)*JTJ(1,0);

            if (std::abs(det) < 1e-12)
                break;
            
            Vec2r ds = Vec2r(
                JTJ(1,1)*res(0) - JTJ(0,1)*res(1),
                -JTJ(1,0)*res(0) + JTJ(0,0)*res(1)
            ) / det;

            s1 = std::clamp(s1 + damping*ds[0], 0.0, 1.0);
            s2 = std::clamp(s2 + damping*ds[1], 0.0, 1.0);

            std::cout << "New s1, s2: " << s1 << ", " << s2 << std::endl;

        }

        // coordinate descent
        // for (int iter = 0; iter < num_iters; iter++)
        // {
        //     // solve for s1 with s2 fixed
        //     Vec3r p1 = elem1->position(s1);
        //     Vec3r dp1 = elem1->dposition_dshat(s1);
        //     Vec3r d2p1 = elem1->d2position_dshat2(s1);

        //     Vec3r p2 = elem2->position(s2);

        //     Real g1 = (p1 - p2).dot(dp1);
        //     Real dg1 = dp1.dot(dp1) + (p1 - p2).dot(d2p1);
        //     if (std::abs(dg1) > 1e-12)
        //         s1 = std::clamp(s1 - g1/dg1, Real(0.0), Real(1.0));

        //     // solve for s2 with s1 fixed
        //     p1 = elem1->position(s1);
        //     Vec3r dp2 = elem2->dposition_dshat(s2);
        //     Vec3r d2p2 = elem2->d2position_dshat2(s2);

        //     Real g2 = (p1 - p2).dot(dp2);
        //     Real dg2 = -dp2.dot(dp2) + (p1 - p2).dot(d2p2);
        //     if (std::abs(dg2) > 1e-12)
        //         s2 = std::clamp(s2 - g2/dg2, Real(0.0), Real(1.0));

        //     std::cout << "New s1, s2: " << s1 << ", " << s2 << std::endl;
        // }

        std::cout << "\nFinal s1 and s2: " << s1 << ", " << s2 << std::endl;
        std::cout << "Distance: " << (elem1->position(s1) - elem2->position(s2)).norm() << std::endl;

        seeds[i].first = s1;
        seeds[i].second = s2;
    }

    
    return seeds;

}   

};

} // namespace Collision