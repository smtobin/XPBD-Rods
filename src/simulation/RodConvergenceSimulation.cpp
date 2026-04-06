#include "simulation/RodConvergenceSimulation.hpp"

#include "common/GaussQuadratureHelper.hpp"

#include <thread>
#include <fstream>

namespace Sim
{

RodConvergenceSimulation::RodConvergenceSimulation()
    : Simulation()
{}

RodConvergenceSimulation::RodConvergenceSimulation(const Config::RodConvergenceSimulationConfig& config)
    : Simulation(config),
    _output_strains(config.outputStrains()),
     _rod_E(config.rodE()), _rod_nu(config.rodNu()), _rod_density(config.rodDensity()),
     _rod_dia(config.rodDia()), _rod_length(config.rodLength()),
    _rigid_body_elements(config.rigidBodyRodElements()),
    _linear_rod_elements(config.linearRodElements()),
    _quadratic_rod_elements(config.quadraticRodElements()),
    _cubic_elements(config.cubicElements()),
    _cubic_hermite_rod_elements(config.cubicHermiteElements())
{

}

void RodConvergenceSimulation::setup()
{
    Simulation::setup();

    // create rods for each number of nodes specified
    for (const auto& num_elem: _rigid_body_elements)
    {
        std::string name = "rigid_body_rod" + std::to_string(num_elem);
        Config::RodConfig config(
            name, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
            Config::RodElementType::RIGID_BODY, true, false, true,
            _rod_length, _rod_dia, num_elem, _rod_density, _rod_E, _rod_nu
        );

        _addObjectFromConfig(config);
    }

    for (const auto& num_elem : _linear_rod_elements)
    {
        std::string name = "linear_rod" + std::to_string(num_elem);
        Config::RodConfig config(
            name, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
            Config::RodElementType::LINEAR, true, false, true,
            _rod_length, _rod_dia, num_elem, _rod_density, _rod_E, _rod_nu
        );

        _addObjectFromConfig(config);
    }

    for (const auto& num_elem : _quadratic_rod_elements)
    {
        std::string name = "quadratic_rod" + std::to_string(num_elem);
        Config::RodConfig config(
            name, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
            Config::RodElementType::QUADRATIC, true, false, true,
            _rod_length, _rod_dia, num_elem, _rod_density, _rod_E, _rod_nu
        );
        _addObjectFromConfig(config);
    }

    for (const auto& num_elem : _cubic_elements)
    {
        std::string name = "cubic_rod" + std::to_string(num_elem);
        Config::RodConfig config(
            name, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
            Config::RodElementType::CUBIC, true, false, true,
            _rod_length, _rod_dia, num_elem, _rod_density, _rod_E, _rod_nu
        );
        _addObjectFromConfig(config);
    }

    for (const auto& num_elem : _cubic_hermite_rod_elements)
    {
        std::string name = "cubic_hermite_rod" + std::to_string(num_elem);
        Config::RodConfig config(
            name, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
            Config::RodElementType::CUBIC_HERMITE, true, false, true,
            _rod_length, _rod_dia, num_elem, _rod_density, _rod_E, _rod_nu
        );
        _addObjectFromConfig(config);
    }

    // create ground truth rod
    Config::RodConfig ground_truth_config(
        "rod", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
        Config::RodElementType::QUADRATIC, true, false, true,
        _rod_length, _rod_dia, 1000, _rod_density, _rod_E, _rod_nu
    );
    _addObjectFromConfig(ground_truth_config);

    _ground_truth = _objects.get<std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<2>>>>().back().get();

}

void RodConvergenceSimulation::update()
{
    Simulation::update();

    // after the sim is finished, we assume that we've reached steady state
    // compute the energy norm between each rod and the final rod

    // std::cout << "\n=== Energy Norms ===" << std::endl;
    // _objects.for_each_element<
    //     std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<0>>>,
    //     std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<1>>>,
    //     std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<2>>>
    //     >([&] (auto& obj)
    // {
    //     std::cout << "Energy norm for " << obj->name() << ": " << _energyNorm(obj.get(), _ground_truth) << std::endl;
    // });

    std::cout << "\n\n=== Position Norms ===" << std::endl;
    _objects.for_each_element<
        std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<0>>>,
        std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<1>>>,
        std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<2>>>,
        std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<3>>>,
        std::unique_ptr<SimObject::XPBDCubicHermiteRod>
        >([&] (auto& obj)
    {
        Vec3r gt_tip = _ground_truth->nodes().back().position;
        std::cout << "Position norm for " << obj->name() << ": " << (obj->nodes().back().position - gt_tip).norm() << std::endl;
    });

    if (_output_strains)
    {
        std::ofstream out_file("strains.txt");

        // write headers
        _objects.for_each_element<
            std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<0>>>,
            std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<1>>>,
            std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<2>>>,
            std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<3>>>,
            std::unique_ptr<SimObject::XPBDCubicHermiteRod>
            >([&] (auto& obj)
        {
            out_file << obj->name() << "_v1 " << obj->name() << "_v2 " << obj->name() << "_v3 " << obj->name() << "_u1 " << obj->name() << "_u2 " << obj->name() << "_u3 ";
        });
        out_file << std::endl;

        // sample strains along the rod
        int num_samples = 10000;
        for (int i = 0; i < num_samples; i++)
        {
            Real s = Real(i) / (num_samples-1);

            _objects.for_each_element<
                std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<0>>>,
                std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<1>>>,
                std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<2>>>,
                std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<3>>>,
                std::unique_ptr<SimObject::XPBDCubicHermiteRod>
                >([&] (auto& obj)
            {
                int elem_ind = std::clamp(static_cast<int>(s * obj->elements().size()), 0, static_cast<int>(obj->elements().size()-1));
                Real s_hat = s * obj->elements().size() - (Real)elem_ind;
                Vec6r strain = obj->elements()[elem_ind].strain(s_hat);
                // if ((s > 0.33 && s < 0.34) || (s > 0.66 && s < 0.67))
                // {
                //     std::cout << obj->name() << " strain at s=" << s << ": " << strain.transpose() << std::endl;
                // }

                out_file << strain[0] << " " << strain[1] << " " << strain[2] << " " << strain[3] << " " << strain[4] << " " << strain[5] << " ";
            });
            out_file << std::endl;
        }
        out_file.close();
    }
}

int RodConvergenceSimulation::run()
{
    // setup if we haven't already
    if (!_setup)
        setup();
    

    // start update thread
    _graphics_scene.displayWindow();

    std::thread update_thread;
    if (_config.simMode() != Config::SimulationMode::FRAME_BY_FRAME)
        update_thread = std::thread(&RodConvergenceSimulation::update, this);
    
    _graphics_scene.interactorStart();
}

template <typename ElementType1, typename ElementType2>
Real RodConvergenceSimulation::_energyNorm(SimObject::XPBDRod_<ElementType1>* rod1, SimObject::XPBDRod_<ElementType2>* rod2)
{
    // take rod2 to be the master rod that we are comparing against

    const auto& rod1_elements = rod1->elements();
    const auto& rod2_elements = rod2->elements();

    // compute shear modulus
    Real G = _rod_E / (2 * (1+_rod_nu));

    // compute cross section properties
    Real radius = _rod_dia/2.0;
    Real area = M_PI * radius * radius;
    Real Ix = M_PI * radius * radius * radius * radius / 4.0;
    Real Iz = 2*Ix;

    // stiffness matrix
    Vec6r stiffness(G*area, G*area, _rod_E*area, _rod_E*Ix, _rod_E*Ix, G*Iz);

    // element rest length
    Real rod1_elem_length = rod1_elements[0].restLength();
    Real rod2_elem_length = rod2_elements[0].restLength();

    // iterate through rod1's elements and compare strains at Gauss points
    Real total_energy_norm = 0;

    auto points = GaussQuadratureHelper<ElementType1::NumGP>::points();
    auto weights = GaussQuadratureHelper<ElementType2::NumGP>::weights();
    for (unsigned i = 0; i < rod1_elements.size(); i++)
    {
        

        for (unsigned j = 0; j < points.size(); j++)
        {
            // get position along rod (i.e. get s)
            Real s = i*rod1_elem_length + points[j]*rod1_elem_length;

            // get index of corresponding element in rod2
            int rod2_elem_ind = static_cast<int>(s / rod2_elem_length);
            Real rod2_shat = (s - rod2_elem_ind*rod2_elem_length) / (rod2_elem_length);

            // compute difference in strain variables at this point
            Vec6r strain1 = rod1_elements[i].strain(points[j]);
            Vec6r strain2 = rod2_elements[rod2_elem_ind].strain(rod2_shat);

            Vec6r strain_diff = strain2 - strain1;

            // std::cout << "\nExp strain " << i << ": \t" << strain2.transpose() << std::endl;
            // std::cout << "GT strain " << i << ": \t" << strain1.transpose() << std::endl; 
            // std::cout << "strain diff " << i << ": \t" << strain_diff.transpose() << std::endl;

            Real energy_norm = rod1_elem_length * weights[j] * strain_diff.transpose() * stiffness.asDiagonal() * strain_diff;
            total_energy_norm += energy_norm;
        }
        
    }
    total_energy_norm = std::sqrt(total_energy_norm);

    return total_energy_norm;
}

} // namespace Sim