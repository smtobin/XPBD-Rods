#include "simobject/group/Plectoneme.hpp"

namespace SimObject
{

Plectoneme::Plectoneme(const Config::PlectonemeConfig& config)
    : XPBDObjectGroup_Base(config)
{
    _cur_twist = 0;
    _cur_displacement = 0;

    _max_twist = config.revolutions() * 2 * M_PI;
    _max_displacement = 0.6;
}

void Plectoneme::setup()
{
    Config::RodConfig rod_config(
        "plectoneme", Vec3r(0,1,0), Vec3r(0,90,0), Vec3r(0,0,0), Vec3r(0,0,0), true,
        Config::RodElementType::QUADRATIC, true, true, true,
        1, 0.01, 10, 1000, 5e6, 0.48, Vec3r(0,0,0)
    );

    rod_config.renderConfig().setColor(Vec3r(1.0, 0.0, 0.0));
    rod_config.renderConfig().setRoughness(0.2);

    auto& rod = _objects.template emplace_back<XPBDRod_<RodElement<2>>>(rod_config);
    rod.setup();

    // store pointer to rod for convenience
    _rod = &rod;
}

void Plectoneme::velocityUpdate(Real dt)
{
    XPBDObjectGroup_Base::velocityUpdate(dt);

    // update the fixed joint constraints for the rod
    std::visit([&](auto& rod) {
        auto& fixed_joints = rod->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>();

        // if we haven't reached the max twist, update the twist angle
        if (_cur_twist < _max_twist)
        {
            Real twist_increment = 0.5*dt;

            Mat3r cur_rot = fixed_joints.back().referenceOrientation();
            fixed_joints.back().setReferenceOrientation(Math::Plus_SO3(cur_rot, Vec3r(0, 0, twist_increment)));

            _cur_twist += twist_increment;

            std::cout << "Cur twist: " << _cur_twist << std::endl;
        }
        else if (_cur_displacement < _max_displacement)
        {
            Real pos_increment = dt/100;

            Vec3r cur_pos = fixed_joints.back().referencePosition();
            fixed_joints.back().setReferencePosition(cur_pos - Vec3r(pos_increment, 0, 0));

            _cur_displacement += pos_increment;
        }
        
    }, _rod);

    // auto& motor_constraints = _constraints.template get<Constraint::RevoluteJointVelocityMotorConstraint>();
    // for (auto& motor_constraint : motor_constraints)
    // {
    //     motor_constraint.setVelocity(std::min(Real(_motor_angular_velocity), motor_constraint.velocity() + 1000*dt));
    //     // std::cout << "Motor velocity: " << motor_constraint.velocity() << std::endl;
    //     motor_constraint.updateTarget(dt);
    // }
}


} // namespace SimObject