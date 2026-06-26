#include "simobject/group/HexBug.hpp"

namespace SimObject
{

HexBug::HexBug(const Config::HexBugConfig& config)
    : XPBDObjectGroup_Base(config),
    _body_initial_position(config.bodyInitialPosition()),
    _body_size(config.bodySize()),
    _body_density(config.bodyDensity()),
    _motor_angular_velocity(config.motorAngularVelocity()),
    _leg_stiffness(config.legStiffness()),
    _leg_length(config.legLength()),
    _leg_length_increment(config.legLengthIncrement()),
    _leg_diameter(config.legDiameter()),
    _leg_curvature(config.legCurvature()),
    _leg_color(config.legColor()),
    _body_color(config.bodyColor())
{

}

void HexBug::setup()
{

    _objects.template reserve<XPBDRigidBox>(5);

    // create "body" with just a box - 4 cm long, 1.25 cm wide, with a little thickness
    Config::XPBDRigidBoxConfig body_config(
        "hexbug_body", _body_initial_position, Vec3r(0,0,0), Vec3r::Zero(), Vec3r::Zero(), true,
        _body_density, false, _body_size
    );

    Config::MeshRenderConfig body_plastic_mesh_config(
        Config::ObjectRenderConfig::RenderType::PBR,
        "../resource/meshes/hexbug_plastic_bottom.STL",
        std::nullopt, std::nullopt, std::nullopt,
        0, 0.5, 0.0, _body_color,
        false,
        true, false,
        Vec3r(0,-2.5e-3,0), Vec3r(0,-90,0), 1e-3*Vec3r::Ones()
    );
    body_config.addRenderMeshConfig(body_plastic_mesh_config);

    Config::MeshRenderConfig body_rim_mesh_config(
        Config::ObjectRenderConfig::RenderType::PBR,
        "../resource/meshes/hexbug_plastic_top.STL",
        std::nullopt, std::nullopt, std::nullopt,
        0, 0.2, 1.0, _leg_color,
        false,
        true, false,
        Vec3r(0,-1.5e-3,0), Vec3r(0,-90,0), 1e-3*Vec3r::Ones()
    );
    body_config.addRenderMeshConfig(body_rim_mesh_config);

    auto& body = _objects.template emplace_back<XPBDRigidBox>(body_config);
    std::cout << "Body mass: " << body.com().mass*1000 << " grams" << std::endl;

    


    // create legs out of 12 rods - 6 on each side of body
    int num_legs_per_side = 6;
    std::vector<Vec3r> body_joint_pos(2*num_legs_per_side);
    for (int side = 0; side < 2; side++)
    {
        Real dx = side == 0 ? -_body_size[0]/2 + _leg_diameter/2 : _body_size[0]/2 - _leg_diameter/2;
        for (int i = 0; i < 6; i++)
        {
            Vec3r pos_local = Vec3r(dx, -_body_size[1]/2, -7*_body_size[2]/16 + _leg_diameter/2 + (2*_body_size[2]/3-_leg_diameter)/(num_legs_per_side-1)*i);
            // Vec3r pos_local = Vec3r(dx, -_body_size[1]/2, -_body_size[2]/2 + _leg_diameter/2 + (_body_size[2]- _leg_diameter)/(num_legs_per_side-1)*i);
            Vec3r leg_base = _body_initial_position + pos_local;
            std::cout << "leg_base: " << leg_base.transpose() << std::endl;
            Config::RodConfig leg_config(
                "hexbug_leg", leg_base, Vec3r(90,10,0), Vec3r(0,0,0), Vec3r(0,0,0), true,
                Config::RodElementType::LINEAR, false, false, true,
                _leg_length - _leg_length_increment*i, _leg_diameter, 1, 1000, _leg_stiffness, 0.4, 0, _leg_curvature
            );
            leg_config.renderConfig().setColor(_leg_color);
            leg_config.renderConfig().setRoughness(0.2);
            leg_config.renderConfig().setCenterlineSamples(15);

            auto& leg = _objects.template emplace_back<XPBDRod_<RodElement<2>>>(leg_config);
            

            body_joint_pos[side*num_legs_per_side + i] = pos_local;
        }
    }

    // add fixed constraints
    auto& legs = _objects.template get<XPBDRod_<RodElement<2>>>();
    // reserve fixed constraints so that pointers to them are valid
    auto& fixed_joint_constraints = _internal_constraints.template get<Constraint::FixedJointConstraint>();
    fixed_joint_constraints.reserve(2*num_legs_per_side);

    for (unsigned i = 0; i < legs.size(); i++)
    {
        auto& leg = legs[i];
        leg.setup();

        // add fixed constraint between leg base and its initial position on the body
        Constraint::FixedJointConstraint fixed_constraint(
            &leg.nodes().front(), Vec3r::Zero(), Mat3r::Identity(),
            &body.com(), body_joint_pos[i], Math::RotMatFromXYZEulerAngles(Vec3r(90,10,0))
        );
        // _internal_constraints.push_back(std::move(fixed_constraint));
        _internal_constraints.push_back(std::move(fixed_constraint));
        leg.setFixedBaseConstraint(&fixed_joint_constraints.back());

        // leg.constraints().for_each_element([&](const auto& constraint) {
        //     _internal_constraints.push_back(constraint);
        //     // _constraints.push_back(constraint);
        // });
    }

    // create eccentric rotating mass
    Vec3r mass_size(0.004, 0.002, 0.004);
    Vec3r mass_ang_velocity(0,0,100);
    Vec3r mass_position_loc = Vec3r(0, -3e-3, 10e-3);
    Config::XPBDRigidBoxConfig eccentric_mass_config(
        "hexbug_eccentric_mass", _body_initial_position + mass_position_loc, Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(), false,
        5400, false, mass_size
    );
    eccentric_mass_config.renderConfig().setColor(Vec3r(0.7,0.7,0.7));
    eccentric_mass_config.renderConfig().setMetallic(1.0);
    auto& ecc_mass = _objects.template emplace_back<XPBDRigidBox>(eccentric_mass_config);
    std::cout << "Eccentric mass: " << ecc_mass.com().mass *1000 << " grams" << std::endl;
    
    // create revolute joint joining eccentric mass to body
    Constraint::RevoluteJointConstraint rev_constraint(
        &body.com(), mass_position_loc + Vec3r(0,mass_size[1]/2,0), Mat3r::Identity(),
        &ecc_mass.com(), Vec3r(0,mass_size[1]/2,0), Mat3r::Identity()
    );
    _constraints.push_back(std::move(rev_constraint));

    // create revolute motor joint constraint
    const auto& rev_ref = _constraints.template get<Constraint::RevoluteJointConstraint>().back();
    Constraint::RevoluteJointVelocityMotorConstraint motor_constraint(
        rev_ref, 0
    );
    _constraints.push_back(std::move(motor_constraint));

    // create motor mass
    Vec3r motor_size(0.004, 0.004, 0.004);
    Vec3r motor_position_loc = Vec3r(0, 0, -10e-3);
    Config::XPBDRigidBoxConfig motor_mass_config(
        "hexbug_motor_mass", _body_initial_position + motor_position_loc, Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(), false,
        5400, false, mass_size
    );
    motor_mass_config.renderConfig().setColor(Vec3r(0.7,0.7,0.7));
    motor_mass_config.renderConfig().setMetallic(1.0);
    auto& motor_mass = _objects.template emplace_back<XPBDRigidBox>(motor_mass_config);

    // create fixed joint joining eccentric mass to body
    Constraint::FixedJointConstraint motor_fixed_constraint(
        &body.com(), motor_position_loc, Mat3r::Identity(),
        &motor_mass.com(), Vec3r(0,0,0), Mat3r::Identity()
    );
    _constraints.push_back(std::move(motor_fixed_constraint));
}

void HexBug::velocityUpdate(Real dt)
{
    XPBDObjectGroup_Base::velocityUpdate(dt);

    auto& motor_constraints = _constraints.template get<Constraint::RevoluteJointVelocityMotorConstraint>();
    for (auto& motor_constraint : motor_constraints)
    {
        motor_constraint.setVelocity(std::max(Real(-_motor_angular_velocity), motor_constraint.velocity() - 1000*dt));
        std::cout << "Motor velocity: " << motor_constraint.velocity() << std::endl;
        motor_constraint.updateTarget(dt);
    }
}

// void HexBug::internalConstraintSolve(Real dt)
// {
//     std::unordered_map<SimObject::OrientedParticle*, int> particle_to_index;

//     // Step 1: calculate total number of constraints and assemble particle -> index map
//     int num_constraints = 0;
//     _internal_constraints.for_each_element([&](const auto& constraint) {
//         using ConstraintType = base_type_t<decltype(constraint)>;
//         num_constraints += ConstraintType::ConstraintDim;

//         for (int i = 0; i < ConstraintType::NumOrientedParticles; i++)
//         {
//             SimObject::OrientedParticle* particle_i = const_cast<SimObject::OrientedParticle*>(constraint.orientedParticles()[i]); // cast away constness - gross! but necessary for now
//             particle_to_index.insert({particle_i, particle_to_index.size()});   // insert if the particle not already accounted for
//         }
//     });

//     // Step 2: assemble global inertia vector
//     VecXr M_inv = VecXr::Zero(6*particle_to_index.size());
//     for (const auto& [particle, index] : particle_to_index)
//     {
//         M_inv.block<3,1>(index*6, 0) = Vec3r::Ones() / particle->mass;
//         M_inv.block<3,1>(index*6+3, 0) = Vec3r(1/particle->Ib[0], 1/particle->Ib[1], 1/particle->Ib[2]);
//     }
    
//     // Step 3: iterate through constraints
//     VecXr C_vec = VecXr::Zero(num_constraints);
//     VecXr alpha = VecXr::Zero(num_constraints);
//     VecXr lambda = VecXr::Zero(num_constraints);
//     MatXr delC = MatXr::Zero(num_constraints, 6*particle_to_index.size());
//     for (int i = 0; i < 1; i++)
//     {
//         int constraint_index = 0;
//         _internal_constraints.for_each_element([&](const auto& constraint) {
//             using ConstraintType = base_type_t<decltype(constraint)>;

//             // evaluate the constraint and put it in global constraint vector
//             typename ConstraintType::ConstraintVecType vec = constraint.evaluate();
//             C_vec.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = vec;

//             alpha.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = constraint.alpha();

//             // evaluate the gradient and put it in global delC matrix
//             typename ConstraintType::GradientMatType gradient = constraint.gradient();
//             for (int i = 0; i < ConstraintType::NumOrientedParticles; i++)
//             {
//                 int particle_index = particle_to_index[constraint.orientedParticles()[i]];
//                 delC.template block<ConstraintType::ConstraintDim, 6>(constraint_index, 6*particle_index) = 
//                     gradient.template block<ConstraintType::ConstraintDim, 6>(0, 6*i);
//             }

//             constraint_index += ConstraintType::ConstraintDim;
//         });

//         // Step 4: assemble and solve
//         // compute LHS
//         std::cout << "Computing LHS..." << std::endl;
//         VecXr alpha_tilde = alpha/(dt*dt);
//         MatXr LHS_mat = delC * M_inv.asDiagonal() * delC.transpose();
//         LHS_mat.diagonal() += alpha_tilde;

//         VecXr RHS_vec = -C_vec - alpha_tilde.asDiagonal() * lambda;

//         std::cout << "Computing LLT..." << std::endl;
//         Eigen::LLT<MatXr> llt(LHS_mat);
//         VecXr dlam = llt.solve(RHS_vec);
//         VecXr dx = M_inv.asDiagonal() * delC.transpose() * dlam;

//         lambda += dlam;

//         // Step 5: update positions
//         for (auto& [particle, index] : particle_to_index)
//         {
//             const Vec3r dp = dx( Eigen::seqN(6*index,3) );
//             const Vec3r dor = dx( Eigen::seqN(6*index+3,3) );
//             particle->positionUpdate(dp, dor);
//         }
//     }
// }

} // namespace SimObject