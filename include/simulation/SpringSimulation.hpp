#pragma once

#include "simulation/Simulation.hpp"

namespace Sim
{

class SpringSimulation : public Simulation
{
public:
    explicit SpringSimulation();

    explicit SpringSimulation(const Config::SimulationConfig& sim_config);

    virtual ~SpringSimulation() = default;

    virtual void setup() override;

protected:
    virtual void _timeStep() override;

private:
    using RigidBodyRod = SimObject::XPBDRod_<SimObject::RodElement<0>>;
    using LinearRod = SimObject::XPBDRod_<SimObject::RodElement<1>>;
    using QuadraticRod = SimObject::XPBDRod_<SimObject::RodElement<2>>;
    using CubicRod = SimObject::XPBDRod_<SimObject::RodElement<3>>;

    std::vector<std::variant<RigidBodyRod*, LinearRod*, QuadraticRod*, CubicRod*>> _rods;
    std::vector<Vec3r> _rod_initial_tips;
    Real _displacement = 0;
};

} // namespace Sim