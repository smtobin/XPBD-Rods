#pragma once

#include "simulation/Simulation.hpp"

// #include "config/SuturingSimulationConfig.hpp"

namespace Sim
{

class SuturingSimulation : public Simulation
{
public:
    using LinearRod = SimObject::XPBDRod_<SimObject::RodElement<1>>;
    using QuadraticRod = SimObject::XPBDRod_<SimObject::RodElement<2>>;
    using CubicRod = SimObject::XPBDRod_<SimObject::RodElement<3>>;

    explicit SuturingSimulation();

    explicit SuturingSimulation(const Config::SimulationConfig& sim_config);

    virtual ~SuturingSimulation() = default;

    virtual void setup() override;

    virtual void notifyKeyPressed(const std::string& key) override;
    virtual void notifyKeyReleased(const std::string& key) override;

    void setStraightToolGrasping(bool state);
    void setCurvedToolGrasping(bool state);

    void updateStraightToolGraspPose(const Vec3r& new_pos, const Mat3r& new_rot);
    void updateCurvedToolGraspPose(const Vec3r& new_pos, const Mat3r& new_rot);
    
    void updateStraightToolPose(const Vec3r& new_pos, const Mat3r& new_rot);
    void updateCurvedToolPose(const Vec3r& new_pos, const Mat3r& new_rot);

protected:
    virtual void _timeStep() override;

    void _updateToolPositionsFromKeyboard();
    void _findClosestPointOnRod(LinearRod* rod, const Vec3r& p, int& element_ind, Real& s_hat, Real& dist);

private:
    bool _straight_tool_grasping = false;
    Mat3r _straight_tool_grasp_rot_offset;
    Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>* _straight_tool_rod_constraint = nullptr;
    LinearRod* _straight_tool_grasped_rod;
    SimObject::XPBDRigidMesh* _straight_tool = nullptr;
    Vec3r _straight_tool_tip_offset;
    SimObject::XPBDRigidSphere* _straight_tool_grasp_sphere;

    bool _curved_tool_grasping = false;
    Mat3r _curved_tool_grasp_rot_offset;
    Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>* _curved_tool_rod_constraint = nullptr;
    LinearRod* _curved_tool_grasped_rod;
    SimObject::XPBDRigidMesh* _curved_tool = nullptr;
    Vec3r _curved_tool_tip_offset;
    SimObject::XPBDRigidSphere* _curved_tool_grasp_sphere;
    
    LinearRod* _thread1;
    LinearRod* _thread2;
};

} // namespace Sim