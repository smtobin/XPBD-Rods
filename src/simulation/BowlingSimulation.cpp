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
    Vec3r pin_size(0.035, 0.23, 0.035);
    Real pin_spacing = 0.2;
    Vec3r front_pin_pos = Vec3r(0, pin_size[1]/2, 0);

    Real string_length = 1;
    Real string_dia = 0.005;
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
            2000,
            false,
            pin_size
        );
        pin_config.renderConfig().setColor(Vec3r(0.3, 0.3, 0.3));
        _addObjectFromConfig(pin_config);
        auto pin = _objects.template get<std::unique_ptr<SimObject::XPBDRigidBox>>().back().get();
        _pins.push_back(pin);
        
        Vec3r pin_top_pos = pos + Vec3r(0, pin_size[1]/2, 0);
        Vec3r string_base_pos = pin_top_pos + Vec3r(0, string_length, 0);
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
            25,
            1150,
            1e7,
            0.4,
            1e-3,
            Vec3r(0,0,0)
        );
        _addObjectFromConfig(string_config);
        auto string = _objects.template get<std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<1>>>>().back().get();
        _strings.push_back(string);

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
        Vec3r::Zero(),
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
    _ball = _objects.template get<std::unique_ptr<SimObject::XPBDRigidSphere>>().back().get();
}

void BowlingSimulation::_timeStep()
{
    if (_state == State::LOWERING)
    {
        // lower the strings a little bit
        Real cur_base_y = 0;
        for (auto& string : _strings)
        {
            // get the base fixed constraint
            auto& base_fixed_constraint = string->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>().back();
            Vec3r cur_base_pos = base_fixed_constraint.referencePosition();
            Vec3r new_base_pos = cur_base_pos + 0.1*_dt * Vec3r(0, -1, 0);
            base_fixed_constraint.setReferencePosition(new_base_pos);

            cur_base_y = new_base_pos[1];
        }

        if (cur_base_y < 1.0)
        {
            _state = State::READY;
            
        }
    }

    else if (_state == State::READY)
    {
        _ball->com().lin_velocity = Vec3r(-5, 0, -0.5);
        _state = State::THROWING;
    }
    else if (_state == State::THROWING)
    {
        if (_ball->com().position[0] < -3)
        {
            _state = State::RAISING;
            _ball->com().lin_velocity = Vec3r::Zero();
            _ball->com().prev_lin_velocity = Vec3r::Zero();
            _ball->com().ang_velocity = Vec3r::Zero();
            _ball->com().prev_ang_velocity = Vec3r::Zero();
            _ball->com().position = Vec3r(5, _ball->radius(), 0.5);
            _ball->com().prev_position = _ball->com().position;
        }
    }
    else if (_state == State::RAISING)
    {
        // raise the strings a little bit
        Real cur_base_y = 0;
        for (auto& string : _strings)
        {
            // get the base fixed constraint
            auto& base_fixed_constraint = string->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>().back();
            Vec3r cur_base_pos = base_fixed_constraint.referencePosition();
            Vec3r new_base_pos = cur_base_pos + 0.1*_dt * Vec3r(0, 1, 0);
            base_fixed_constraint.setReferencePosition(new_base_pos);

            cur_base_y = new_base_pos[1];
        }

        if (cur_base_y > 2.0)
        {
            _state = State::LOWERING;
        }
    }

    Simulation::_timeStep();
}

} // namespace Sim