#include "simulation/SuturingSimulation.hpp"

namespace Sim
{

SuturingSimulation::SuturingSimulation()
    : Simulation()
{}

SuturingSimulation::SuturingSimulation(const Config::SimulationConfig& config)
    : Simulation(config)
{}

void SuturingSimulation::setup()
{
    Simulation::setup();

    // reserve space
    _objects.template reserve<std::unique_ptr<LinearRod>>(2);

    // create threads
    Config::RodConfig thread1_config(
        "thread1",
        Vec3r(0, 1, 0.15),
        Vec3r(0, 0, 0),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        true,
        0.5,
        0.2,
        Config::RodElementType::LINEAR,
        true,
        false,
        true,
        125,    // length
        0.2,    // diameter
        300,    // num elements
        1.1e-6, // density
        2000,   // E
        0.4,    // nu
        1e5,    // beta
        Vec3r(0,0,0)    // curvature
    );
    thread1_config.renderConfig().setColor(Vec3r(1.0, 0.0, 0.0));
    _addObjectFromConfig(thread1_config);
    _thread1 = _objects.template get<std::unique_ptr<LinearRod>>().back().get();

    Config::RodConfig thread2_config(
        "thread2",
        Vec3r(0, 1, -0.15),
        Vec3r(0, 180, 0),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        true,
        0.5,
        0.2,
        Config::RodElementType::LINEAR,
        true,
        false,
        true,
        65,    // length
        0.2,    // diameter
        100,    // num elements
        1.1e-6, // density
        2000,   // E
        0.4,    // nu
        1e5,    // beta
        Vec3r(0,0,0)    // curvature
    );
    thread2_config.renderConfig().setColor(Vec3r(0.0, 1.0, 0.0));
    _addObjectFromConfig(thread2_config);
    _thread2 = _objects.template get<std::unique_ptr<LinearRod>>().back().get();

    // update the fixed base constraints on thread1 and thread2 so that they go into the ground
    auto& base_fixed_constraint1 = _thread1->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>().back();
    base_fixed_constraint1.setReferencePosition(Vec3r(0, 0.01, 1.0));
    base_fixed_constraint1.setReferenceOrientation(Math::RotMatFromXYZEulerAngles(Vec3r(-90,0,0)));
    auto& base_fixed_constraint2 = _thread2->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>().back();
    base_fixed_constraint2.setReferencePosition(Vec3r(0, 0.01, -1.0));
    base_fixed_constraint2.setReferenceOrientation(Math::RotMatFromXYZEulerAngles(Vec3r(-90,0,0)));
}

} // namespace Sim