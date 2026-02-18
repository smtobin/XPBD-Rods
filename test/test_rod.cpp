#include "simobject/rod/XPBDRod.hpp"
#include "config/RodConfig.hpp"
#include "collision/CollisionScene.hpp"

#include <variant>
#include <unordered_map>

#define DT 1e-1

template<typename T>
struct base_type { using type = T; };

template<typename T>
struct base_type<T*> : base_type<T> {};

template<typename T>
struct base_type<T&> : base_type<T> {};

template<typename T>
struct base_type<T&&> : base_type<T> {};

template<typename T>
struct base_type<const T> : base_type<T> {};

template<typename T>
struct base_type<volatile T> : base_type<T> {};

template<typename T>
using base_type_t = typename base_type<T>::type;

void solveGlobalXPBDSystem(const std::vector<XPBDConstraints_ConstPtrVariantType>& constraints)
{

    std::unordered_map<SimObject::OrientedParticle*, int> particle_to_index;

    // Step 1: calculate total number of constraints and assemble particle -> index map
    int num_constraints = 0;
    for (const auto& constraint_variant : constraints)
    {
        std::visit([&](const auto& constraint) {
            using ConstraintType = base_type_t<decltype(constraint)>;
            num_constraints += ConstraintType::ConstraintDim;

            for (int i = 0; i < ConstraintType::NumParticles; i++)
            {
                SimObject::OrientedParticle* particle_i = const_cast<SimObject::OrientedParticle*>(constraint->particles()[i]); // cast away constness - gross! but necessary for now
                particle_to_index.insert({particle_i, particle_to_index.size()});   // insert if the particle not already accounted for
            }
        },
        constraint_variant);
    }

    // Step 2: assemble global inertia vector
    VecXr M_inv = VecXr::Zero(6*particle_to_index.size());
    for (const auto& [particle, index] : particle_to_index)
    {
        M_inv.block<3,1>(index*6, 0) = Vec3r::Ones() / particle->mass;
        M_inv.block<3,1>(index*6+3, 0) = Vec3r(1/particle->Ib[0], 1/particle->Ib[1], 1/particle->Ib[2]);
    }
    
    // Step 3: iterate through constraints
    VecXr C_vec = VecXr::Zero(num_constraints);
    VecXr alpha = VecXr::Zero(num_constraints);
    VecXr lambda = VecXr::Zero(num_constraints);
    MatXr delC = MatXr::Zero(num_constraints, 6*particle_to_index.size());
    for (int i = 0; i < 1; i++)
    {
        int constraint_index = 0;
        for (const auto& constraint_variant : constraints)
        {
            std::visit([&](const auto& constraint) {
                using ConstraintType = base_type_t<decltype(constraint)>;

                // evaluate the constraint and put it in global constraint vector
                typename ConstraintType::ConstraintVecType vec = constraint->evaluate();
                C_vec.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = vec;

                alpha.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = constraint->alpha();

                // evaluate the gradient and put it in global delC matrix
                typename ConstraintType::GradientMatType gradient = constraint->gradient();
                for (int i = 0; i < ConstraintType::NumParticles; i++)
                {
                    int particle_index = particle_to_index[constraint->particles()[i]];
                    delC.template block<ConstraintType::ConstraintDim, 6>(constraint_index, 6*particle_index) = 
                        gradient.template block<ConstraintType::ConstraintDim, 6>(0, 6*i);
                }

                constraint_index += ConstraintType::ConstraintDim;
            },
            constraint_variant);
        }

        // Step 4: assemble and solve
        // compute LHS
        std::cout << "Computing LHS..." << std::endl;
        VecXr alpha_tilde = alpha/(DT*DT);
        MatXr LHS_mat = delC * M_inv.asDiagonal() * delC.transpose();
        LHS_mat.diagonal() += alpha_tilde;

        VecXr RHS_vec = -C_vec - alpha_tilde.asDiagonal() * lambda;

        std::cout << "Computing LLT..." << std::endl;
        Eigen::LLT<MatXr> llt(LHS_mat);
        VecXr dlam = llt.solve(RHS_vec);
        VecXr dx = M_inv.asDiagonal() * delC.transpose() * dlam;

        lambda += dlam;

        // Step 5: update positions
        for (auto& [particle, index] : particle_to_index)
        {
            const Vec3r dp = dx( Eigen::seqN(6*index,3) );
            const Vec3r dor = dx( Eigen::seqN(6*index+3,3) );
            particle->positionUpdate(dp, dor);
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

int main()
{
    int num_nodes = 21;
    Real dt = 1e-3;
    // create rod
    Config::RodConfig rod_config(
        "test_rod",
        Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(),
        true, true,
        1.0, 0.1,
        num_nodes,
        1000,
        1e8, 0.4
    );

    SimObject::CircleCrossSection rod_xs(
        0.05
    );

    SimObject::XPBDRod rod(rod_config, rod_xs);
    rod.setup();

    auto& rod_nodes = rod.nodes();

    SimObject::OrientedParticle& middle_node = rod_nodes[num_nodes/2];
    std::cout << "Middle node initial position: " << middle_node.position.transpose() << std::endl;
    rod.internalConstraintSolve(dt);
    std::cout << "Middle node position after internalConstraintSolve(): " << middle_node.position.transpose() << std::endl;
    middle_node.position += Vec3r(0.01, 0, 0);
    for (int i = 0; i < 10; i++)
    {
        rod.internalConstraintSolve(dt);
        std::cout << "  Middle node position after internalConstraintSolve() " << i << ": " << middle_node.position.transpose() << std::endl;
    }


    ///////////////////////////////////////////////

    /** Test global solve with two rods + collision constraint */

    Config::RodConfig rod_config1(
        "rod1",
        Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(), Vec3r::Zero(),
        true, true,
        1.0, 0.1,
        10,
        1000,
        1e7, 0.4
    );
    SimObject::XPBDRod rod1(rod_config1, rod_xs);
    rod1.setup();

    Config::RodConfig rod_config2(
        "rod2",
        Vec3r(-0.505, 0.098, 0.505), Vec3r(0,90,0), Vec3r::Zero(), Vec3r::Zero(),
        false, false,
        1.0, 0.1,
        10,
        1000,
        1e6, 0.4
    );
    SimObject::XPBDRod rod2(rod_config2, rod_xs);
    rod2.setup();

    // create global particle list
    std::unordered_map<SimObject::OrientedParticle*, int> particle_to_index;
    for (auto& node : rod1.nodes())
    {
        particle_to_index.insert({&node, particle_to_index.size()});
    }
    for (auto& node : rod2.nodes())
    {
        particle_to_index.insert({&node, particle_to_index.size()});
    }

    // create collision constraint(s)
    std::vector<Constraint::RodRodCollisionConstraint> collision_constraints;
    Collision::CollisionScene collision_scene;
    collision_scene.addObject(&rod1);
    collision_scene.addObject(&rod2);
    auto detected_collisions = collision_scene.detectCollisions();
    for (const auto& detected_collision : detected_collisions)
    {
        std::visit([&](auto&& collision) {
            using T = std::decay_t<decltype(collision)>;
            if constexpr (std::is_same_v<T, Collision::SegmentSegmentCollision>)
            {
                std::cout << "segment-segment collision!" << std::endl;
                std::cout << "  rod1: " << collision.segment1_particle1->position.transpose() << ", " << collision.segment1_particle2->position.transpose() << std::endl;
                std::cout << "  rod2: " << collision.segment2_particle1->position.transpose() << ", " << collision.segment2_particle2->position.transpose() << std::endl;
                // using ConstraintType = Constraint::RodRodCollisionConstraint;
                // auto& constraint_vec = _constraints.template get<ConstraintType>();
                collision_constraints.emplace_back(
                    collision.rod1,
                    collision.segment1_particle1, collision.segment1_particle2, collision.alpha1, collision.radius1, 
                    collision.rod2,
                    collision.segment2_particle1, collision.segment2_particle2, collision.alpha2, collision.radius2,
                    collision.normal
                );
                // ConstVectorHandle<ConstraintType> constraint_ref(&constraint_vec, constraint_vec.size()-1);
                // _solver.addConstraint(constraint_ref);
            }
        }, detected_collision);
    }

    std::cout << "Collision constraint before solve: " << collision_constraints[0].evaluate() << std::endl;
    const auto& particles_in_collision = collision_constraints[0].particles();
    for (const auto& particle : particles_in_collision)
    {
        std::cout << "Original position for particle " << particle << ": " << particle->position.transpose() << std::endl;
    }

    // create and solve global system
    std::vector<SimObject::ConstraintAndLambda> rod1_constraints = rod1.internalConstraintsAndLambdas();
    std::vector<SimObject::ConstraintAndLambda> rod2_constraints = rod2.internalConstraintsAndLambdas();

    std::vector<XPBDConstraints_ConstPtrVariantType> all_constraints;
    for (const auto& constraint_and_lambda : rod1_constraints)
        all_constraints.push_back(constraint_and_lambda.constraint);
    for (const auto& constraint_and_lambda : rod2_constraints)
        all_constraints.push_back(constraint_and_lambda.constraint);
    for (const auto& constraint : collision_constraints)
        all_constraints.push_back(&constraint);

    solveGlobalXPBDSystem(all_constraints);

    // now evaluate collision constraint
    std::cout << "Collision constraint after solve: " << collision_constraints[0].evaluate() << std::endl;

    // evaluate positions of nodes involved in collision constraint
    for (const auto& particle : particles_in_collision)
    {
        std::cout << "New position for particle " << particle << ": " << particle->position.transpose() << std::endl;
    }
}