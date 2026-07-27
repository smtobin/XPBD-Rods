#include "simulation/SpringSimulation.hpp"

namespace Sim
{

SpringSimulation::SpringSimulation()
    : Simulation()
{
}

SpringSimulation::SpringSimulation(const Config::SimulationConfig& config)
    : Simulation(config)
{

}

void SpringSimulation::setup()
{
    Simulation::setup();

    // get all the rods that were created
    auto& rb_rods = _objects.template get<std::unique_ptr<RigidBodyRod>>();
    auto& lin_rods = _objects.template get<std::unique_ptr<LinearRod>>();
    auto& quad_rods = _objects.template get<std::unique_ptr<QuadraticRod>>();
    auto& cub_rods = _objects.template get<std::unique_ptr<CubicRod>>();

    for (auto& rb_rod : rb_rods)
        _rods.push_back(rb_rod.get());
    for (auto& lin_rod : lin_rods)
        _rods.push_back(lin_rod.get());
    for (auto& quad_rod : quad_rods)
        _rods.push_back(quad_rod.get());
    for (auto& cub_rod : cub_rods)
        _rods.push_back(cub_rod.get());

    // get initial tip positions of rods
    for (const auto& rod_variant : _rods)
    {
        std::visit([&](const auto& rod) {
            _rod_initial_tips.push_back(rod->nodes().back().position);
        }, rod_variant);
    }
}

void SpringSimulation::_timeStep()
{
    Simulation::_timeStep();

    if (_time > 3 && std::abs(_displacement) < 0.01)
        _displacement -= 0.01*_dt;

    // go through rods and update fixed joint constraint
    for (unsigned i = 0; i < _rods.size(); i++)
    {
        std::visit([&](auto& rod) {
            // get internal constraints and lambdas
            std::vector<SimObject::ConstraintAndLambda> constraints_and_lambdas = rod->internalConstraintsAndLambdas();
            // compute force + moment from tip constraint
            std::visit([&](auto& tip_constraint) {
                if constexpr(std::is_same_v<base_type_t<decltype(tip_constraint)>, Constraint::OneSidedFixedJointConstraint>)
                {
                    const Vec6r lambda = Eigen::Map<const Vec6r>(constraints_and_lambdas.back().lambda);
                    Vec6r force = tip_constraint->gradient().transpose() * lambda / (_dt*_dt);
                    std::cout << "Tip force: " << force.transpose() << std::endl;
                }
                
            }, constraints_and_lambdas.back().constraint);

            // get fixed joint constraint at tip (assumes that the base is also fixed)
            auto& fixed_joint_constraints = rod->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>();
            auto& tip_fixed_constraint = fixed_joint_constraints.back();
            tip_fixed_constraint.setReferencePosition(_rod_initial_tips[i] + Vec3r(0, _displacement, 0) );
        }, _rods[i]);
    }
}

} // namespace Sim