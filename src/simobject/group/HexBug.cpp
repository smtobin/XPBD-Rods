#include "simobject/group/HexBug.hpp"

namespace SimObject
{

HexBug::HexBug(const Config::HexBugConfig& config)
    : XPBDObjectGroup_Base(config)
{

}

void HexBug::setup()
{
    Real body_length = 0.04;
    Real body_width = 0.0125;
    Real body_thickness = 0.005;

    Vec3r body_center(0,0.03,0);

    Real leg_radius = body_width/10.0;
    Real leg_length = 0.01875;

    // create "body" with just a box - 4 cm long, 1.25 cm wide, with a little thickness
    Config::XPBDRigidBoxConfig body_config(
        "hexbug_body", body_center, Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(), true,
        1030, false, Vec3r(body_width, body_thickness, body_length)
    );
    auto& body = _objects.template emplace_back<XPBDRigidBox>(body_config);
    std::cout << "Body mass: " << body.com().mass*1000 << " grams" << std::endl;

    // create legs out of 12 rods - 6 on each side of body
    int num_legs_per_side = 6;
    std::vector<Vec3r> body_joint_pos(2*num_legs_per_side);
    for (int side = 0; side < 2; side++)
    {
        Real dx = side == 0 ? -body_width/2 + leg_radius : body_width/2 - leg_radius;
        for (int i = 0; i < 6; i++)
        {
            Vec3r pos_local = Vec3r(dx, -body_thickness/2, -body_length/2 + leg_radius + (body_length-2*leg_radius)/(num_legs_per_side-1)*i);
            Vec3r leg_base = body_center + pos_local;
            std::cout << "leg_base: " << leg_base.transpose() << std::endl;
            Config::RodConfig leg_config(
                "hexbug_leg", leg_base, Vec3r(90,10,0), Vec3r(0,0,0), Vec3r(0,0,0), true,
                Config::RodElementType::LINEAR, false, false, true,
                leg_length, leg_radius*2, 5, 1000, 1e6, 0.4, Vec3r(10,0,0)
            );
            auto& leg = _objects.template emplace_back<XPBDRod_<RodElement<1>>>(leg_config);
            

            body_joint_pos[side*num_legs_per_side + i] = pos_local;
        }
    }

    // add fixed constraints
    auto& legs = _objects.template get<XPBDRod_<RodElement<1>>>();
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