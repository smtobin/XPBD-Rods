#include "simobject/group/Plectoneme.hpp"

namespace SimObject
{

Plectoneme::Plectoneme(const Config::PlectonemeConfig& config)
    : XPBDObjectGroup_Base(config)
{
    _cur_twist = 0;
    _cur_displacement = 0;

    _max_twist = config.revolutions() * 2 * M_PI;
    _max_displacement = 6;
}

void Plectoneme::setup()
{
    Config::RodConfig rod_config(
        "plectoneme", Vec3r(0,10,0), Vec3r(0,90,0), Vec3r(0,0,0), Vec3r(0,0,0), true,
        Config::RodElementType::QUADRATIC, true, true, true,
        10, 0.1, 40, 1000, 5e7, 0.4, Vec3r(0,0,0)
    );

    rod_config.renderConfig().setColor(Vec3r(1.0, 0.0, 0.0));
    rod_config.renderConfig().setRoughness(0.2);

    auto& rod = _objects.template emplace_back<XPBDRod_<RodElement<1>>>(rod_config);
    rod.setup();

    // store pointer to rod for convenience
    _rod = &rod;
}

// void Plectoneme::internalConstraintSolve(Real dt)
// {
//     std::unordered_map<SimObject::OrientedParticle*, int> particle_to_index;

//     const auto& rod_internal_constraints = _objects.template get<SimObject::XPBDRod_<SimObject::RodElement<1>>>().back().internalConstraints();

//     XPBDConstraints_ConstPtrContainer internal_constraints;
//     rod_internal_constraints.for_each_element([&](const auto& constraint) {
//         using ConstraintType = base_type_t<decltype(constraint)>;
//         if (constraint.isInequality())
//         {
//             // evaluate the inequality constraint to see if it is active
//             typename ConstraintType::ConstraintVecType vec = constraint.evaluate();
//             if (vec[0] < 0)
//             {
//                 internal_constraints.push_back(&constraint);
//             }
//         }
//         else
//         {
//             internal_constraints.push_back(&constraint);
//         }
//     });

//     // Step 1: calculate total number of constraints and assemble particle -> index map
//     int num_constraints = 0;
//     internal_constraints.for_each_element([&](const auto& constraint) {
//             using ConstraintType = base_type_t<decltype(constraint)>;
//             num_constraints += ConstraintType::ConstraintDim;

//             for (int i = 0; i < ConstraintType::NumOrientedParticles; i++)
//             {
//                 SimObject::OrientedParticle* particle_i = const_cast<SimObject::OrientedParticle*>(constraint->orientedParticles()[i]); // cast away constness - gross! but necessary for now
//                 particle_to_index.insert({particle_i, particle_to_index.size()});   // insert if the particle not already accounted for
//             }
//         });

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
//     int constraint_index = 0;
//     internal_constraints.for_each_element([&](const auto& constraint) {
//             using ConstraintType = base_type_t<decltype(constraint)>;

//             // evaluate the constraint and put it in global constraint vector
//             typename ConstraintType::ConstraintVecType vec = constraint->evaluate();
//             C_vec.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = vec;

//             alpha.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = constraint->alpha();

//             // evaluate the gradient and put it in global delC matrix
//             typename ConstraintType::GradientMatType gradient = constraint->gradient();
//             for (int i = 0; i < ConstraintType::NumOrientedParticles; i++)
//             {
//                 int particle_index = particle_to_index[constraint->orientedParticles()[i]];
//                 delC.template block<ConstraintType::ConstraintDim, 6>(constraint_index, 6*particle_index) = 
//                     gradient.template block<ConstraintType::ConstraintDim, 6>(0, 6*i);
//             }

//             constraint_index += ConstraintType::ConstraintDim;
//     });

//     // Step 4: assemble and solve
//     // compute LHS
//     // std::cout << "Computing LHS..." << std::endl;
//     VecXr alpha_tilde = alpha/(dt*dt);
//     MatXr LHS_mat = delC * M_inv.asDiagonal() * delC.transpose();
//     LHS_mat.diagonal() += alpha_tilde;

//     VecXr RHS_vec = -C_vec - alpha_tilde.asDiagonal() * lambda;

//     // std::cout << "Computing LLT..." << std::endl;
//     Eigen::LLT<MatXr> llt(LHS_mat);
//     VecXr dlam = llt.solve(RHS_vec);
//     VecXr dx = M_inv.asDiagonal() * delC.transpose() * dlam;

//     lambda += dlam;

//     // Step 5: update positions
//     for (auto& [particle, index] : particle_to_index)
//     {
//         const Vec3r dp = dx( Eigen::seqN(6*index,3) );
//         const Vec3r dor = dx( Eigen::seqN(6*index+3,3) );
//         particle->positionUpdate(dp, dor);
//     }
    
// }

void Plectoneme::velocityUpdate(Real dt)
{
    XPBDObjectGroup_Base::velocityUpdate(dt);

    // update the fixed joint constraints for the rod
    std::visit([&](auto& rod) {
        auto& fixed_joints = rod->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>();

        // if we haven't reached the max twist, update the twist angle
        if (_cur_displacement < _max_displacement)
        {
            Real pos_increment = dt;

            Vec3r cur_pos = fixed_joints.back().referencePosition();
            fixed_joints.back().setReferencePosition(cur_pos - Vec3r(pos_increment, 0, 0));

            _cur_displacement += pos_increment;
        }
        else if (_cur_twist < _max_twist)
        {
            Real twist_increment = dt;

            Mat3r cur_rot = fixed_joints.back().referenceOrientation();
            fixed_joints.back().setReferenceOrientation(Math::Plus_SO3(cur_rot, Vec3r(0, 0, twist_increment)));

            _cur_twist += twist_increment;

            std::cout << "Cur twist: " << _cur_twist << std::endl;
        }
        
        

        std::cout << "Total rotation: " << rod->totalRotation().transpose() << std::endl;
        
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