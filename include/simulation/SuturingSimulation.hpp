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

    virtual void notifyKeyPressed(const std::string& key) override;
    virtual void notifyKeyReleased(const std::string& key) override;

protected:
    virtual void _timeStep() override;

    void _updateToolPositionsFromKeyboard();

private:
    bool _straight_tool_grasping = false;
    Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>* _straight_tool_rod_constraint = nullptr;
    SimObject::XPBDRigidMesh* _straight_tool = nullptr;
    Vec3r _straight_tool_tip_offset;
    SimObject::XPBDRigidSphere* _straight_tool_grasp_sphere;

    bool _curved_tool_grasping = false;
    Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>* _curved_tool_rod_constraint = nullptr;
    SimObject::XPBDRigidMesh* _curved_tool = nullptr;
    Vec3r _curved_tool_tip_offset;
    SimObject::XPBDRigidSphere* _curved_tool_grasp_sphere;

    using LinearRod = SimObject::XPBDRod_<SimObject::RodElement<1>>;
    using QuadraticRod = SimObject::XPBDRod_<SimObject::RodElement<2>>;
    using CubicRod = SimObject::XPBDRod_<SimObject::RodElement<3>>;
    LinearRod* _thread1;
    LinearRod* _thread2;
};

} // namespace Sim