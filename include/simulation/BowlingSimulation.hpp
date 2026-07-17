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

protected:
    virtual void _timeStep() override;

private:
    enum class State
    {
        LOWERING,
        READY,
        THROWING,
        RAISING,
        FIXING
    };
    State _state = State::LOWERING;
    Real _state_change_time = 0;

    std::vector<SimObject::XPBDRod_<SimObject::RodElement<3>>*> _strings;
    std::vector<SimObject::XPBDRigidBox*> _pins;
    SimObject::XPBDRigidSphere* _ball;

    /** Parameters */
    Real _string_length = 1.5;
};

} // namespace Sim