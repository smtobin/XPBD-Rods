#pragma once

#include "common/common.hpp"

#include "simulation/SimulationLogger.hpp"

#include "simobject/rod/XPBDRod.hpp"
#include "graphics/RodGraphicsObject.hpp"
#include "graphics/GraphicsScene.hpp"

#include "config/SimulationConfig.hpp"
#include "config/SimulationRenderConfig.hpp"

#include "simobject/rod/XPBDRod.hpp"
#include "simobject/rod/XPBDCubicHermiteRod.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/group/XPBDPendulum.hpp"
#include "simobject/group/XPBDConcentricTubeRobot.hpp"
#include "simobject/group/HexBug.hpp"
#include "simobject/group/RPSRobot.hpp"
#include "simobject/group/Plectoneme.hpp"

#include "solver/GaussSeidelSolver.hpp"

#include "collision/CollisionScene.hpp"

#include <vector>
#include <deque>
#include <functional>
#include <atomic>
#include <unordered_map>


namespace Sim
{

class Simulation
{
    public:
    explicit Simulation();

    explicit Simulation(const Config::SimulationConfig& sim_config);

    virtual ~Simulation() = default;

    virtual void setup();

    virtual int run();

    virtual void update();

    VecXr primaryResidual() const;
    VecXr constraintResidual() const;


    // event handling
    virtual void notifyKeyPressed(const std::string& key);
    virtual void notifyKeyReleased(const std::string& key);
    virtual void notifyMouseMoved(double mx, double my);
    virtual void notifyLeftMouseButtonPressed();
    virtual void notifyLeftMouseButtonReleased();

    template <class ConstraintType, class ...Args>
    void addConstraint(Args&&... args)
    {
        auto& constraint_vec = _constraints.template get<ConstraintType>();
        constraint_vec.emplace_back(std::forward<Args>(args)...);
        ConstVectorHandle<ConstraintType> constraint_ref(&constraint_vec, constraint_vec.size()-1);
        _solver.addConstraint(constraint_ref);
    }

    protected:

    template <typename ObjectType>
    void _addConstraintsFromObject(ObjectType* obj, Config::XPBDObjectConfig::ProjectorType proj_type = Config::XPBDObjectConfig::ProjectorType::BLOCK)
    {
        const XPBDConstraints_Container& constraints = obj->constraints();
        constraints.for_each_element_indexed([&](const auto& constraint, size_t index) {
            // do some gymnastics to get the vector we're currently on
            using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;
            const auto& single_type_constraint_vec = constraints.template get<ConstraintType>();
            // create a VectorHandle
            ConstVectorHandle<ConstraintType> handle(&single_type_constraint_vec, index);
            _solver.addConstraint(handle, proj_type);
        });

        // iterate through any joint constraints and make sure the CollisionScene ignores collisions appropriately
        const XPBDConstraints_Container& internal_constraints = obj->internalConstraints();
        constraints.for_each_element(XPBDTwoSidedJointConstraints_TypeList{}, [&](auto& constraint) {
            _collision_scene.addJoint(constraint.orientedParticles()[0], constraint.orientedParticles()[1]);
        });
        internal_constraints.for_each_element(XPBDTwoSidedJointConstraints_TypeList{}, [&](auto& constraint) {
            _collision_scene.addJoint(constraint.orientedParticles()[0], constraint.orientedParticles()[1]);
        });
    }

    template <typename ElementType>
    void _addConstraintsFromObject(SimObject::XPBDRod_<ElementType>* rod, Config::XPBDObjectConfig::ProjectorType proj_type = Config::XPBDObjectConfig::ProjectorType::BLOCK)
    {
        const XPBDConstraints_Container& constraints = rod->internalConstraints();
        constraints.for_each_element_indexed([&](const auto& constraint, size_t index) {
            // do some gymnastics to get the vector we're currently on
            using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;
            const auto& single_type_constraint_vec = constraints.template get<ConstraintType>();
            // create a VectorHandle
            ConstVectorHandle<ConstraintType> handle(&single_type_constraint_vec, index);
            _solver.addConstraint(handle, proj_type);
        });
    }

    template<typename ConfigType>        
    void _addObjectFromConfig(const ConfigType& obj_config)
    {
        // using ObjPtrType = std::unique_ptr<typename ConfigType::SimObjectType>;
        using ObjType = typename ConfigType::SimObjectType;
        using ObjPtrType = std::unique_ptr<ObjType>;
        ObjType* new_obj_ptr = nullptr;
        if constexpr (std::is_base_of_v<SimObject::XPBDObjectGroup_Base, ObjType>)
        {
            _object_groups.template push_back<ObjPtrType>(std::make_unique<ObjType>(obj_config));
            new_obj_ptr = _object_groups.template get<ObjPtrType>().back().get();
            new_obj_ptr->setup();

            

            // add the ObjectGroup's constraints to the solver
            _addConstraintsFromObject(new_obj_ptr, obj_config.projectorType());
        }
        else
        {
            _objects.template emplace_back<ObjPtrType>(std::make_unique<ObjType>(obj_config));
            new_obj_ptr = _objects.template get<ObjPtrType>().back().get();
            new_obj_ptr->setup();
        }

        _graphics_scene.addObject(new_obj_ptr, obj_config);
        

        // assign global indices to the new object's nodes
        // NOTE: this assumes thatobjects are not deleted during the course of the sim
        std::vector<const SimObject::OrientedParticle*> obj_particles = new_obj_ptr->particles();
        for (const auto& particle : obj_particles)
        {
            _particle_ptr_to_index.insert({particle, _particle_ptr_to_index.size()});
        }
        // resize the vectors for storing the inertially predicted positions/orientations
        _p_tilde.resize(_particle_ptr_to_index.size());
        _R_tilde.resize(_particle_ptr_to_index.size());

        // if the particles of this object should be logged, create logging outputs for them
        if (obj_config.logParticles() && _logger)
        {
            for (unsigned i = 0; i < obj_particles.size(); i++)
            {
                const std::string var_name = new_obj_ptr->name() + "_particle" + std::to_string(i);
                _logger->addOutput(var_name, obj_particles[i]);
            }
        }
    }

    void _addObjectFromConfig(const Config::RodConfig& rod_config)
    {
        SimObject::XPBDObject_Base* new_obj_ptr = nullptr;
        if (rod_config.elementType() == Config::RodElementType::RIGID_BODY)
        {
            // // create the old XPBDRod
            // SimObject::CircleCrossSection cross_section(rod_config.diameter()/2.0, 20);
            // _objects.template push_back<std::unique_ptr<SimObject::XPBDRod>>(std::make_unique<SimObject::XPBDRod>(rod_config, cross_section));
            // SimObject::XPBDRod* new_rod_ptr = _objects.template get<std::unique_ptr<SimObject::XPBDRod>>().back().get();
            // new_rod_ptr->setup();

            // // add new rod to graphics scene to be visualized
            // _graphics_scene.addObject(new_rod_ptr, rod_config.renderConfig());

            // new_obj_ptr = new_rod_ptr;

            using ElementType = SimObject::RodElement<0>;
            _objects.template push_back<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>(std::make_unique<SimObject::XPBDRod_<ElementType>>(rod_config));
            SimObject::XPBDRod_<ElementType>* new_rod_ptr = _objects.template get<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>().back().get();
            new_rod_ptr->setup();

            // add constraints to Gauss-Seidel solver if not using a global solve on the internal rod constraints
            if (!new_rod_ptr->globalSolve())
            {
                _addConstraintsFromObject(new_rod_ptr);
            }

            // add new rod to graphics scene to be visualized
            _graphics_scene.addObject(new_rod_ptr, rod_config);

            new_obj_ptr = new_rod_ptr;
        }
        else if (rod_config.elementType() == Config::RodElementType::LINEAR)
        {  
            using ElementType = SimObject::RodElement<1>;
            _objects.template push_back<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>(std::make_unique<SimObject::XPBDRod_<ElementType>>(rod_config));
            SimObject::XPBDRod_<ElementType>* new_rod_ptr = _objects.template get<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>().back().get();
            new_rod_ptr->setup();

            // add constraints to Gauss-Seidel solver if not using a global solve on the internal rod constraints
            if (!new_rod_ptr->globalSolve())
            {
                _addConstraintsFromObject(new_rod_ptr);
            }

            // add new rod to graphics scene to be visualized
            _graphics_scene.addObject(new_rod_ptr, rod_config);

            new_obj_ptr = new_rod_ptr;
        }
        else if (rod_config.elementType() == Config::RodElementType::QUADRATIC)
        {
            using ElementType = SimObject::RodElement<2>;
            _objects.template push_back<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>(std::make_unique<SimObject::XPBDRod_<ElementType>>(rod_config));
            SimObject::XPBDRod_<ElementType>* new_rod_ptr = _objects.template get<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>().back().get();
            new_rod_ptr->setup();

            // add constraints to Gauss-Seidel solver if not using a global solve on the internal rod constraints
            if (!new_rod_ptr->globalSolve())
            {
                _addConstraintsFromObject(new_rod_ptr);
            }

            // add new rod to graphics scene to be visualized
            _graphics_scene.addObject(new_rod_ptr, rod_config);

            new_obj_ptr = new_rod_ptr;
        }
        else if (rod_config.elementType() == Config::RodElementType::CUBIC)
        {
            using ElementType = SimObject::RodElement<3>;
            _objects.template push_back<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>(std::make_unique<SimObject::XPBDRod_<ElementType>>(rod_config));
            SimObject::XPBDRod_<ElementType>* new_rod_ptr = _objects.template get<std::unique_ptr<SimObject::XPBDRod_<ElementType>>>().back().get();
            new_rod_ptr->setup();

            // add constraints to Gauss-Seidel solver if not using a global solve on the internal rod constraints
            if (!new_rod_ptr->globalSolve())
            {
                _addConstraintsFromObject(new_rod_ptr);
            }

            // add new rod to graphics scene to be visualized
            _graphics_scene.addObject(new_rod_ptr, rod_config);

            new_obj_ptr = new_rod_ptr;
        }
        else if (rod_config.elementType() == Config::RodElementType::CUBIC_HERMITE)
        {
            _objects.template push_back<std::unique_ptr<SimObject::XPBDCubicHermiteRod>>(std::make_unique<SimObject::XPBDCubicHermiteRod>(rod_config));
            SimObject::XPBDCubicHermiteRod* new_rod_ptr = _objects.template get<std::unique_ptr<SimObject::XPBDCubicHermiteRod>>().back().get();
            new_rod_ptr->setup();

            // add new rod to graphics scene to be visualized
            _graphics_scene.addObject((SimObject::XPBDRod_<SimObject::CubicHermiteRodElement>*)new_rod_ptr, rod_config);

            new_obj_ptr = new_rod_ptr;
        }

        // assign global indices to the rod's nodes
        std::vector<const SimObject::OrientedParticle*> obj_particles = new_obj_ptr->particles();
        for (const auto& particle : obj_particles)
        {
            _particle_ptr_to_index.insert({particle, _particle_ptr_to_index.size()});
        }
        // resize the vectors for storing the inertially predicted positions/orientations
        _p_tilde.resize(_particle_ptr_to_index.size());
        _R_tilde.resize(_particle_ptr_to_index.size());

        // if the particles of this object should be logged, only log the first and last particle (rods may have many particles)
        if (rod_config.logParticles() && _logger)
        {
            for (unsigned i = 0; i < obj_particles.size(); i++)
            {
                const std::string var_name = new_obj_ptr->name() + "_particle" + std::to_string(i);
                _logger->addOutput(var_name, obj_particles[i]);
            }
            // const std::string var_name_0 = new_obj_ptr->name() + "_particle0";
            // const std::string var_name_end = new_obj_ptr->name() + "_particle" + std::to_string(obj_particles.size()-1);
            // _logger->addOutput(var_name_0, obj_particles[0]);
            // _logger->addOutput(var_name_end, obj_particles.back());   
        }
    }

    SimObject::XPBDRigidBody_Base* _findRigidBodyWithName(const std::string& name);
    void _addJointFromConfig(const Config::FixedJointConfig& joint_config);
    void _addJointFromConfig(const Config::RevoluteJointConfig& joint_config);
    void _addJointFromConfig(const Config::PrismaticJointConfig& joint_config);
    void _addJointFromConfig(const Config::SphericalJointConfig& joint_config);

    void _timeStep();

    void _updateGraphics();

    template<typename CallbackT>
    void _addCallback(CallbackT&& lambda)
    {
        std::function<void()> wrapper = [lambda = std::forward<CallbackT>(lambda)]() {
            lambda();
        };

        _callback_queue.push_back(std::move(wrapper));
    }

    /** Collision detection + processing */

    /** Runs collision detection */
    void _detectCollisions();
    /** Adds a collision constraint for a rigid-rigid collision */
    void _processCollision(const Collision::RigidRigidCollision& collision);
    /** Adds a collision constraint for a rigid-rod collision */
    void _processCollision(const Collision::RigidSegmentCollision& collision);
    /** Adds a collision constraint for a rod-rod collision */
    void _processCollision(const Collision::SegmentSegmentCollision& collision);

    /** Templated helper method for handling a rigid-rod collision. */
    template <int Order>
    void _processRodRigidBodyCollision(SimObject::RodElement<Order>* elem, const Collision::RigidSegmentCollision& collision);

    /** Templated helper method for converting RodElement_Base to the appropriate derived RodElement class,
     * so as to create the appropriate collision constraint.
     */
    template <int Order>
    void _processRodRodCollision(SimObject::RodElement<Order>* elem1, Real s_hat1, const Vec3r& cp_local1, 
                                 SimObject::RodElement_Base* elem2, Real s_hat2, const Vec3r& cp_local2,
                                 Vec3r& normal, Real mu_s, Real mu_d);

    protected:
    bool _setup;

    Real _time;
    Real _dt;
    Real _end_time;
    Real _g_accel;
    int _viewer_refresh_time_ms;

    XPBDConstraints_Container _constraints;
    XPBDObjects_UniquePtrContainer _objects;
    XPBDObjectGroups_UniquePtrContainer _object_groups;

    /** Maps pointers to paritcles to global indices. Used primarily for computing the primary residual. */
    std::unordered_map<const SimObject::OrientedParticle*, int> _particle_ptr_to_index;

    /** Store the intertially predicted positions + orientations. Useful for computing the primary residual. */
    std::vector<Vec3r> _p_tilde;
    std::vector<Mat3r> _R_tilde;

    int _solver_iters = 1;
    Solver::GaussSeidelSolver _solver;

    std::deque<std::function<void()>> _callback_queue;

    /** Responsible for logging various simulation quantities. */
    std::unique_ptr<SimulationLogger> _logger;

    // graphics
    Graphics::GraphicsScene _graphics_scene;

    // collisions
    Collision::CollisionScene _collision_scene;

    Config::SimulationConfig _config;
};

} // namespace Simulation
