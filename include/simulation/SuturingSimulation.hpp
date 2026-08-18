#pragma once

#include "simulation/Simulation.hpp"

// #include "config/SuturingSimulationConfig.hpp"

namespace Sim
{

class SuturingSimulation : public Simulation
{
public:
    explicit SuturingSimulation();

    explicit SuturingSimulation(const Config::SimulationConfig& sim_config);

    virtual ~SuturingSimulation() = default;

    virtual void setup() override;

protected:
    // virtual void _timeStep() override;

private:
    bool _straight_tool_grasping;
    Constraint::OneSidedFixedJointConstraint* _straight_tool_rod_constraint;

    bool _curved_tool_grasping;
    Constraint::OneSidedFixedJointConstraint* _curved_tool_rod_constraint;

    using LinearRod = SimObject::XPBDRod_<SimObject::RodElement<1>>;
    LinearRod* _thread1;
    LinearRod* _thread2;
};

} // namespace Sim