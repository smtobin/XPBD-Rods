#include "simulation/BowlingSimulation.hpp"

namespace Sim
{

BowlingSimulation::BowlingSimulation()
    : Simulation()
{

}

BowlingSimulation::BowlingSimulation(const Config::SimulationConfig& config)
    : Simulation(config)
{

}

void BowlingSimulation::setup()
{
    Simulation::setup();

    // reserve space
    _objects.template reserve<std::unique_ptr<SimObject::XPBDRigidBox>>(20);
    _constraints.template reserve<Constraint::FixedJointConstraint>(20);

    // create the pins
    int num_pins_in_row = 1;
    int index_in_row = 0;
    int num_pins = 10;
    Vec3r pin_size(0.05, 0.23, 0.05);
    Real pin_spacing = 0.2;
    Vec3r front_pin_pos = Vec3r(0, pin_size[1]/2, 0);

    Real string_length = 1;
    Real string_dia = 0.002;
    for (int i = 0; i < num_pins; i++)
    {
        int num_spaces = num_pins_in_row - 1;
        Vec3r row_start_pos = front_pin_pos + Vec3r(-pin_spacing * num_spaces, 0, -pin_spacing * num_spaces/2.0);
        Vec3r pos = row_start_pos + index_in_row * Vec3r(0,0,pin_spacing);
        Config::XPBDRigidBoxConfig pin_config(
            "pin" + std::to_string(i),
            pos,
            Vec3r::Zero(),
            Vec3r::Zero(),
            Vec3r::Zero(),
            true,
            0.5,
            0.2,
            1000,
            false,
            pin_size
        );
        pin_config.renderConfig().setColor(Vec3r(0.3, 0.3, 0.3));
        _addObjectFromConfig(pin_config);
        auto& pin = _objects.template get<std::unique_ptr<SimObject::XPBDRigidBox>>().back();
        
        Vec3r pin_top_pos = pos + Vec3r(0, pin_size[1]/2, 0);
        Vec3r string_base_pos = pin_top_pos + Vec3r(0, 1.5*string_length, 0);
        Config::RodConfig string_config(
            "pin" + std::to_string(i) + "string",
            string_base_pos,
            Vec3r(90, 0, 0),
            Vec3r(0,0,0),
            Vec3r(0,0,0),
            true,
            0.5,
            0.2,
            Config::RodElementType::LINEAR,
            true,
            false,
            true,
            string_length,
            string_dia,
            20,
            1150,
            1e9,
            0.4,
            1e2,
            Vec3r(0,0,0)
        );
        _addObjectFromConfig(string_config);
        auto& string = _objects.template get<std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<1>>>>().back();

        /** TODO: make this cleaner? */
        auto& fixed_joint_constraints = _constraints.template get<Constraint::FixedJointConstraint>();
        Constraint::FixedJointConstraint string_pin_constraint(
            &string->nodes().back(), Vec3r::Zero(), Mat3r::Identity(),
            &pin->com(), Vec3r(0, pin_size[1]/2, 0), Math::RotMatFromXYZEulerAngles(Vec3r(90,0,0))
        );
        fixed_joint_constraints.push_back(std::move(string_pin_constraint));
        ConstVectorHandle<Constraint::FixedJointConstraint> constraint_ref(&fixed_joint_constraints, fixed_joint_constraints.size()-1);
        _solver.addConstraint(constraint_ref);

        string->setFixedTipConstraint(&fixed_joint_constraints.back());

        index_in_row++;
        if (index_in_row == num_pins_in_row)
        {
            index_in_row = 0;
            num_pins_in_row++;
        }
    }

    // create the ball
    Real ball_radius = 0.07;
    Config::XPBDRigidSphereConfig ball_config(
        "ball",
        Vec3r(front_pin_pos[0] + 5, ball_radius, 0.6),
        Vec3r::Zero(),
        Vec3r(-5, 0, -0.5),
        Vec3r::Zero(),
        true,
        0.5,
        0.2,
        2000,
        false,
        ball_radius
    );
    ball_config.renderConfig().setColor(Vec3r(0,0,1.0));
    _addObjectFromConfig(ball_config);
    auto& ball = _objects.template get<std::unique_ptr<SimObject::XPBDRigidSphere>>().back();
}

} // namespace Sim