#pragma once

#include "simulation/Simulation.hpp"

namespace Sim
{

class RodGraspingSimulation : public Simulation
{
    public:
    explicit RodGraspingSimulation();

    explicit RodGraspingSimulation(const Config::SimulationConfig& sim_config);

    // event handling
    virtual void notifyMouseMoved(double mx, double my) override;
    virtual void notifyLeftMouseButtonPressed() override;

    private:
    void _graspClosestNode();


    private:
    bool _grasping = false;
    SimObject::XPBDRod* _grasped_rod = nullptr;
    const SimObject::OrientedParticle* _grasped_rod_node = nullptr;
    int _grasped_rod_node_index = -1;
    Constraint::OneSidedFixedJointConstraint* _grasping_constraint = nullptr;
    
    int _last_mx;
    int _last_my;
};

} // namespace Sim