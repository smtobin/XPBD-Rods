#pragma once

#include "simulation/Simulation.hpp"

// #include "config/BowlingSimulationConfig.hpp"

namespace Sim
{

class BowlingSimulation : public Simulation
{
public:
    explicit BowlingSimulation();

    explicit BowlingSimulation(const Config::SimulationConfig& sim_config);

    virtual ~BowlingSimulation() = default;

    virtual void setup() override;

private:
};

} // namespace Sim