#include "simulation/RodConvergenceSimulation.hpp"

#include "common/GaussQuadratureHelper.hpp"

#include <thread>

namespace Sim
{

RodConvergenceSimulation::RodConvergenceSimulation()
    : Simulation()
{}

RodConvergenceSimulation::RodConvergenceSimulation(const Config::RodConvergenceSimulationConfig& config)
    : Simulation(config), _rod_E(config.rodE()), _rod_nu(config.rodNu()), _rod_density(config.rodDensity()),
     _rod_dia(config.rodDia()), _rod_length(config.rodLength()),
    _linear_rod_elements(config.linearRodElements()), _quadratic_rod_elements(config.quadraticRodElements())
{

}

void RodConvergenceSimulation::setup()
{
    Simulation::setup();

    // create rods for each number of nodes specified
    for (const auto& num_elem : _linear_rod_elements)
    {
        std::string name = "linear_rod" + std::to_string(num_elem);
        Config::RodConfig config(
            name, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
            Config::RodElementType::LINEAR, true, false,
            _rod_length, _rod_dia, num_elem, _rod_density, _rod_E, _rod_nu
        );

        _addObjectFromConfig(config);
    }

    for (const auto& num_elem : _quadratic_rod_elements)
    {
        std::string name = "quadratic_rod" + std::to_string(num_elem);
        Config::RodConfig config(
            name, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
            Config::RodElementType::QUADRATIC, true, false,
            _rod_length, _rod_dia, num_elem, _rod_density, _rod_E, _rod_nu
        );
        _addObjectFromConfig(config);
    }

    // create ground truth rod
    Config::RodConfig ground_truth_config(
        "rod", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        Config::RodElementType::QUADRATIC, true, false,
        _rod_length, _rod_dia, 500, _rod_density, _rod_E, _rod_nu
    );
    _addObjectFromConfig(ground_truth_config);

    _ground_truth = _objects.get<std::unique_ptr<SimObject::XPBDRod_<1>>>().back().get();

}

void RodConvergenceSimulation::update()
{
    Simulation::update();

    // after the sim is finished, we assume that we've reached steady state
    // compute the energy norm between each rod and the final rod

    _objects.for_each_element<std::unique_ptr<SimObject::XPBDRod_<1>>, std::unique_ptr<SimObject::XPBDRod_<2>>>([&] (auto& obj)
    {
        std::cout << "Energy norm for " << obj->name() << ": " << _energyNorm(obj.get(), _ground_truth) << std::endl;
    });
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

template <int Order1, int Order2>
Real RodConvergenceSimulation::_energyNorm(SimObject::XPBDRod_<Order1>* rod1, SimObject::XPBDRod_<Order2>* rod2)
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

    // iterate through rod2's elements
    Real total_energy_norm = 0;
    for (unsigned i = 0; i < rod2_elements.size(); i++)
    {
        // get position along rod (i.e. get s)
        Real s = i*rod2_elem_length + rod2_elem_length/2.0;

        // get index of corresponding element in rod1
        int rod1_elem_ind = static_cast<int>(s / rod1_elem_length);
        Real rod1_shat = (s - rod1_elem_ind*rod1_elem_length) / (rod1_elem_length);

        // compute difference in strain variables at this point
        Vec6r strain1 = rod1_elements[rod1_elem_ind].strain(rod1_shat);
        Vec6r strain2 = rod2_elements[i].strain(0.5);

        Vec6r strain_diff = strain2 - strain1;
        Real energy_norm = rod2_elem_length * strain_diff.transpose() * stiffness.asDiagonal() * strain_diff;
        total_energy_norm += energy_norm;
    }
    total_energy_norm = std::sqrt(total_energy_norm);

    return total_energy_norm;
}

} // namespace Sim