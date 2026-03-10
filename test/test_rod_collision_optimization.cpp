#include "common/common.hpp"
#include "common/math.hpp"
#include "simobject/rod/RodElement.hpp"

#include <array>

SimObject::OrientedParticle randomParticle()
{
    Vec3r pos = Vec3r::Random();
    Vec3r R_vec = Vec3r::Random();
    SimObject::OrientedParticle particle;
    particle.position = pos;
    particle.orientation = Math::Exp_so3(R_vec);

    return particle;
}

template <int Order1, int Order2>
void groundTruth(const SimObject::RodElement<Order1>* elem1, const SimObject::RodElement<Order2>* elem2)
{
    int num_samples = 1000;
    std::vector<std::vector<Real>> distance_grid(num_samples);
    for (int i = 0; i < num_samples; i++)
    {
        distance_grid[i].resize(num_samples);
    }

    for (int i = 0; i < num_samples; i++)
    {
        for (int j = 0; j < num_samples; j++)
        {
            Vec3r p1 = elem1->position((Real)i / (num_samples-1));
            Vec3r p2 = elem2->position((Real)j / (num_samples-1));
            distance_grid[i][j] = (p1 - p2).norm();
        }
    }

    for (int i = 1; i < num_samples-1; i++)
    {
        for (int j = 1; j < num_samples-1; j++)
        {
            Real d1 = distance_grid[i+1][j];
            Real d2 = distance_grid[i-1][j];
            Real d3 = distance_grid[i][j+1];
            Real d4 = distance_grid[i][j-1];

            Real d = distance_grid[i][j];
            if (d <= d1 && d <= d2 && d <= d3 && d <= d4)
            {
                std::cout << "Local minimum at s1=" << (Real)i / (num_samples-1) << ", s2=" << (Real)j / (num_samples-1) << ", d=" << d << std::endl;
            }
        }
    }
}

int main()
{
    SimObject::OrientedParticle p1; p1.position = Vec3r(0,0,0);
    SimObject::OrientedParticle p2; p2.position = Vec3r(0,0.4,0.1);
    SimObject::OrientedParticle p3; p3.position = Vec3r(0.2,0.8,0.3);
    SimObject::OrientedParticle p4; p4.position = Vec3r(0.5,0.6,0.2);
    SimObject::OrientedParticle p5; p5.position = Vec3r(0.2,0.5,0.2);
    SimObject::OrientedParticle p6; p6.position = Vec3r(0.1,0.2,0.3);

    std::array<SimObject::OrientedParticle*, 3> elem1_nodes = {&p1, &p3, &p2};
    std::array<SimObject::OrientedParticle*, 3> elem2_nodes = {&p4, &p6, &p5};

    SimObject::RodElement<2> elem1(elem1_nodes, 0.5);
    SimObject::RodElement<2> elem2(elem2_nodes, 0.5);

    SimObject::closestPointsBetweenRodElements(&elem1, &elem2);

    groundTruth(&elem1, &elem2);
}
