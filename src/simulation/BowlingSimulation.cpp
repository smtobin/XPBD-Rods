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
    Vec3r pin_size(0.06, 0.38, 0.06);
    Real pin_spacing = 0.2;
    Vec3r front_pin_pos = Vec3r(0, pin_size[1]/2, 0);

    Real string_length = 1.5;
    Real string_dia = 0.01;
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
            70,
            1150,
            1e7,
            0.4,
            1e2,
            Vec3r(0,0,0)
        );
        string_config.renderConfig().setCenterlineSamples(100);
        // string_config.renderConfig().setDrawFrames(true);
        _addObjectFromConfig(string_config);
        auto string = _objects.template get<std::unique_ptr<SimObject::XPBDRod_<SimObject::RodElement<1>>>>().back().get();
        _strings.push_back(string);
        auto& base_fixed_constraint = string->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>().back();
        base_fixed_constraint.setAlpha(Vec6r(0, 1, 0, 0, 0, 0));

        /** TODO: make this cleaner? */
        // auto& fixed_joint_constraints = _constraints.template get<Constraint::FixedJointConstraint>();
        Constraint::FixedJointConstraint string_pin_constraint(
            &string->nodes().back(), Vec3r::Zero(), Mat3r::Identity(),
            &pin->com(), Vec3r(0, pin_size[1]/2, 0), Math::RotMatFromXYZEulerAngles(Vec3r(90,0,0))
        );
        // string_pin_constraint.setAlpha(Vec6r(0, 0, 0, 0, 0, 1e-2));
        string->template addInternalConstraint<Constraint::FixedJointConstraint>(string_pin_constraint);
        auto& internal_fixed_joint_constraints = string->internalConstraints().template get<Constraint::FixedJointConstraint>();
        // fixed_joint_constraints.push_back(std::move(string_pin_constraint));
        // ConstVectorHandle<Constraint::FixedJointConstraint> constraint_ref(&fixed_joint_constraints, fixed_joint_constraints.size()-1);
        // _solver.addConstraint(constraint_ref);

        string->setFixedTipConstraint(&internal_fixed_joint_constraints.back());
        _collision_scene.addJoint(&string->nodes().back(), &pin->com());

        index_in_row++;
        if (index_in_row == num_pins_in_row)
        {
            index_in_row = 0;
            num_pins_in_row++;
        }

        // if (i >= 5)
        //     break;
    }

    // create the ball
    Real ball_radius = 0.2183/2;
    Config::XPBDRigidSphereConfig ball_config(
        "ball",
        Vec3r(front_pin_pos[0] + 5, ball_radius, 0.01),
        Vec3r::Zero(),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5,
        0.2,
        1250,
        false,
        ball_radius
    );
    ball_config.renderConfig().setColor(Vec3r(0,0,1.0));
    _addObjectFromConfig(ball_config);
    _ball = _objects.template get<std::unique_ptr<SimObject::XPBDRigidSphere>>().back().get();

    // create the backstop and walls
    Vec3r backstop_size(0.05, 1.5, 2);
    Vec3r backstop_pos(front_pin_pos[0] - pin_spacing*6, backstop_size[1]/2 - 0.2, front_pin_pos[2]);
    Config::XPBDRigidBoxConfig backstop_config(
        "backstop",
        backstop_pos,
        Vec3r::Zero(),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5,
        0.2,
        2000,
        true,
        backstop_size
    );
    _addObjectFromConfig(backstop_config);

    Vec3r wall_size(0.05, 1.5, 1.7);
    Vec3r wall1_pos(front_pin_pos[0] - pin_spacing*3, wall_size[1]/2 - 0.2, front_pin_pos[2] - pin_spacing*4);
    Config::XPBDRigidBoxConfig wall1_config(
        "wall1",
        wall1_pos,
        Vec3r(0, 90, 0),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5,
        0.2,
        2000,
        true,
        wall_size
    );
    _addObjectFromConfig(wall1_config);

    Vec3r wall2_pos(front_pin_pos[0] - pin_spacing*3, wall_size[1]/2 - 0.2, front_pin_pos[2] + pin_spacing*4);
    Config::XPBDRigidBoxConfig wall2_config(
        "wall2",
        wall2_pos,
        Vec3r(0, 90, 0),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5,
        0.2,
        2000,
        true,
        wall_size
    );
    _addObjectFromConfig(wall2_config);

    // lane
    Vec3r lane_size(18, 0.1, 1.05);
    Vec3r lane_pos(front_pin_pos[0] - pin_spacing*4 + lane_size[0]/2, -lane_size[1]/2, front_pin_pos[2]);
    Config::XPBDRigidBoxConfig lane_config(
        "lane",
        lane_pos,
        Vec3r::Zero(),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5, 0.1,
        1000,
        true,
        lane_size
    );
    _addObjectFromConfig(lane_config);

    // gutters
    Vec3r gutter_size(lane_size[0], 0.1, 0.23);
    Vec3r left_gutter_pos(lane_pos[0], -lane_size[1] -gutter_size[1]/2, -lane_size[2]/2 - gutter_size[2]/2);
    Vec3r right_gutter_pos(lane_pos[0], -lane_size[1] - gutter_size[1]/2, lane_size[2]/2 + gutter_size[2]/2);
    Config::XPBDRigidBoxConfig left_gutter_config(
        "left_gutter",
        left_gutter_pos,
        Vec3r::Zero(),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5, 0.1,
        1000,
        true,
        gutter_size
    );
    _addObjectFromConfig(left_gutter_config);
    Config::XPBDRigidBoxConfig right_gutter_config(
        "right_gutter",
        right_gutter_pos,
        Vec3r::Zero(),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5, 0.1,
        1000,
        true,
        gutter_size
    );
    _addObjectFromConfig(right_gutter_config);

    // back gutter
    Vec3r back_gutter_size(1, 0.1, lane_size[2] + 2*gutter_size[2]);
    Vec3r back_gutter_pos(lane_pos[0] - lane_size[0]/2 -back_gutter_size[0]/2 , -lane_size[1] - gutter_size[1]/2, lane_pos[2]);
    Config::XPBDRigidBoxConfig back_gutter_config(
        "back_gutter",
        back_gutter_pos,
        Vec3r::Zero(),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5, 0.1,
        1000,
        true,
        back_gutter_size
    );
    _addObjectFromConfig(back_gutter_config);

    // front wall
    Vec3r front_wall_size(0.05, 1.5, 2*back_gutter_size[2]);
    Vec3r front_wall_pos(front_pin_pos[0] + 0.2, 0.7 + front_wall_size[1]/2, lane_pos[2]);
    Config::XPBDRigidBoxConfig front_wall_config(
        "front_wall",
        front_wall_pos,
        Vec3r::Zero(),
        Vec3r::Zero(),
        Vec3r::Zero(),
        true,
        0.5, 0.1,
        1000,
        true,
        front_wall_size
    );
    _addObjectFromConfig(front_wall_config);
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
            Vec3r new_base_pos = cur_base_pos + 0.3*_dt * Vec3r(0, -1, 0);
            base_fixed_constraint.setReferencePosition(new_base_pos);

            cur_base_y = new_base_pos[1];
        }

        if (cur_base_y < _string_length)
        {
            _state = State::READY;
            _state_change_time = _time;
        }
    }

    else if (_state == State::READY)
    {
        _ball->com().lin_velocity = Vec3r(-10, 0, 0);
        _state = State::THROWING;
        _state_change_time = _time;
    }
    else if (_state == State::THROWING)
    {
        if (_time - _state_change_time > 3)
        {
            _state = State::RAISING;
            _state_change_time = _time;

            _ball->com().lin_velocity = Vec3r::Zero();
            _ball->com().prev_lin_velocity = Vec3r::Zero();
            _ball->com().ang_velocity = Vec3r::Zero();
            _ball->com().prev_ang_velocity = Vec3r::Zero();
            _ball->com().position = Vec3r(5, _ball->radius(), 0.01);
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
            Vec3r new_base_pos = cur_base_pos + 0.3*_dt * Vec3r(0, 1, 0);
            base_fixed_constraint.setReferencePosition(new_base_pos);

            Vec6r cur_alpha = base_fixed_constraint.alpha();
            base_fixed_constraint.setAlpha(cur_alpha/(1 + 10*_dt));

            cur_base_y = new_base_pos[1];
        }

        if (cur_base_y > _string_length + 0.6)
        {
            _state = State::FIXING;
            _state_change_time = _time;

            // add elastic fixed constraints to constrain the pins
            // auto& fixed_joint_constraints = _constraints.template get<Constraint::OneSidedFixedJointConstraint>();
            // for (unsigned i = 0; i < _pins.size(); i++)
            // {
            //     auto& pin = _pins[i];
            //     auto& string = _strings[i];

            //     // desired pose is the base pose of the string, translated down the length of the string
            //     Mat3r R_des = string->nodes().back().orientation;
            //     Vec3r p_des = string->nodes().back().position + Vec3r(0,-_string_length,0);

            //     Constraint::OneSidedFixedJointConstraint settle_constraint(
            //         p_des, R_des,
            //         &pin->com(),
            //         Vec3r::Zero(),
            //         Math::RotMatFromXYZEulerAngles(Vec3r(90,0,0)),
            //         1*Vec6r::Ones()
            //     );
            //     fixed_joint_constraints.push_back(std::move(settle_constraint));
            //     ConstVectorHandle<Constraint::OneSidedFixedJointConstraint> constraint_ref(&fixed_joint_constraints, fixed_joint_constraints.size()-1);
            //     _solver.addConstraint(constraint_ref);

            // }
        }
    }
    else if (_state == State::FIXING)
    {
        for (unsigned i = 0; i < _pins.size(); i++)
        {
            auto& pin = _pins[i];
            pin->com().lin_velocity *= 0.999;
            pin->com().ang_velocity *= 0.999;
        }
        if (_time - _state_change_time > 10)
        {
            _state = State::LOWERING;
            _state_change_time = _time;

            // clear fixed constraint projectors
            // _solver.template clearProjectorsOfType<Constraint::OneSidedFixedJointConstraint>();
            // // clear fixed constraints
            // _constraints.template clear_types<Constraint::OneSidedFixedJointConstraint>();
        }
    }

    Simulation::_timeStep();
}

} // namespace Sim