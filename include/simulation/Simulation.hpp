#pragma once

#include "common/common.hpp"

#include "simulation/SimulationLogger.hpp"

#include "simobject/rod/XPBDRod.hpp"
#include "graphics/RodGraphicsObject.hpp"
#include "graphics/GraphicsScene.hpp"

#include "config/SimulationConfig.hpp"
#include "config/SimulationRenderConfig.hpp"

#include "simobject/rod/XPBDRod.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/group/XPBDPendulum.hpp"
#include "simobject/group/XPBDConcentricTubeRobot.hpp"

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

    virtual void setup();

    int run();

    void update();

    VecXr primaryResidual() const;
    VecXr constraintResidual() const;


    // event handling
    virtual void notifyKeyPressed(const std::string& key);
    virtual void notifyKeyReleased(const std::string& key);
    virtual void notifyMouseMoved(double mx, double my);
    virtual void notifyLeftMouseButtonPressed();
    virtual void notifyLeftMouseButtonReleased();

    protected:

    template<typename ConfigType>        
    typename ConfigType::SimObjectType* _addObjectFromConfig(const ConfigType& obj_config)
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

            _graphics_scene.addObject(new_obj_ptr, obj_config.renderConfig());

            // add the ObjectGroup's constraints to the solver
            const XPBDConstraints_Container& constraints = new_obj_ptr->constraints();
            constraints.for_each_element_indexed([&](const auto& constraint, size_t index) {
                // do some gymnastics to get the vector we're currently on
                using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;
                const auto& single_type_constraint_vec = constraints.template get<ConstraintType>();
                // create a VectorHandle
                ConstVectorHandle<ConstraintType> handle(&single_type_constraint_vec, index);
                _solver.addConstraint(handle, obj_config.projectorType());
            });
        }
        else
        {
            _objects.template emplace_back<ObjPtrType>(std::make_unique<ObjType>(obj_config));
            new_obj_ptr = _objects.template get<ObjPtrType>().back().get();
            new_obj_ptr->setup();

            _graphics_scene.addObject(new_obj_ptr, obj_config.renderConfig());
        }

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

        return new_obj_ptr;
    }

    SimObject::XPBDRod* _addObjectFromConfig(const Config::RodConfig& rod_config)
    {
        using RodPtrType = std::unique_ptr<SimObject::XPBDRod>;
        SimObject::CircleCrossSection cross_section(rod_config.diameter()/2.0, 20);
        _objects.template push_back<RodPtrType>(std::make_unique<SimObject::XPBDRod>(rod_config, cross_section));
        SimObject::XPBDRod* new_rod_ptr = _objects.template get<RodPtrType>().back().get();
        new_rod_ptr->setup();

        // add new rod to graphics scene to be visualized
        _graphics_scene.addObject(new_rod_ptr, rod_config.renderConfig());

        // assign global indices to the rod's nodes
        std::vector<const SimObject::OrientedParticle*> obj_particles = new_rod_ptr->particles();
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
            const std::string var_name_0 = new_rod_ptr->name() + "_particle0";
            const std::string var_name_end = new_rod_ptr->name() + "_particle" + std::to_string(obj_particles.size()-1);
            _logger->addOutput(var_name_0, obj_particles[0]);
            _logger->addOutput(var_name_end, obj_particles.back());   
        }

        return new_rod_ptr;
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
