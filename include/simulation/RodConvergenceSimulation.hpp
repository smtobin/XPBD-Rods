#pragma once

#include "simulation/Simulation.hpp"

#include "config/RodConvergenceSimulationConfig.hpp"

namespace Sim
{

class RodConvergenceSimulation : public Simulation
{
public:
    explicit RodConvergenceSimulation();

    explicit RodConvergenceSimulation(const Config::RodConvergenceSimulationConfig& sim_config);

    virtual ~RodConvergenceSimulation() = default;

    virtual void setup() override;

    virtual void update() override;

    virtual int run() override;

private:
    template <typename ElementType1, typename ElementType2>
    Real _energyNorm(SimObject::XPBDRod_<ElementType1>* rod1, SimObject::XPBDRod_<ElementType2>* rod2);
    
    Real _rod_E;
    Real _rod_nu;
    Real _rod_density;
    Real _rod_dia;
    Real _rod_length;

    std::vector<int> _rigid_body_elements;
    std::vector<int> _linear_rod_elements;
    std::vector<int> _quadratic_rod_elements;
    std::vector<int> _cubic_hermite_rod_elements;

    SimObject::XPBDRod_<SimObject::RodElement<2>>* _ground_truth;
};

} // namespace Sim