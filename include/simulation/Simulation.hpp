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
#include "simobject/group/XPBDFourBarLinkage.hpp"
#include "simobject/group/XPBDConcentricTubeRobot.hpp"

#include "solver/GaussSeidelSolver.hpp"

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
        ObjType* new_obj_ptr = nullptr;
        if constexpr (std::is_base_of_v<SimObject::XPBDObjectGroup_Base, ObjType>)
        {
            ObjType& new_obj = _object_groups.template emplace_back<ObjType>(obj_config);
            new_obj.setup();
            _graphics_scene.addObject(&new_obj, obj_config.renderConfig());

            // add the ObjectGroup's constraints to the solver
            const XPBDConstraints_Container& constraints = new_obj.constraints();
            constraints.for_each_element([&](const auto& constraint) {
                _solver.addConstraint(&constraint, obj_config.projectorType());
            });

            new_obj_ptr = &new_obj;
        }
        else
        {
            ObjType& new_obj = _objects.template emplace_back<ObjType>(obj_config);
            new_obj.setup();

            _graphics_scene.addObject(&new_obj, obj_config.renderConfig());

            new_obj_ptr = &new_obj;
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

        return new_obj_ptr;
    }

    SimObject::XPBDRod* _addObjectFromConfig(const Config::RodConfig& rod_config)
    {
        SimObject::CircleCrossSection cross_section(rod_config.diameter()/2.0, 20);
        SimObject::XPBDRod& new_rod = _objects.template emplace_back<SimObject::XPBDRod>(rod_config, cross_section);
        new_rod.setup();

        // add new rod to graphics scene to be visualized
        _graphics_scene.addObject(&new_rod, rod_config.renderConfig());

        // assign global indices to the rod's nodes
        std::vector<const SimObject::OrientedParticle*> obj_particles = new_rod.particles();
        for (const auto& particle : obj_particles)
        {
            _particle_ptr_to_index.insert({particle, _particle_ptr_to_index.size()});
        }
        // resize the vectors for storing the inertially predicted positions/orientations
        _p_tilde.resize(_particle_ptr_to_index.size());
        _R_tilde.resize(_particle_ptr_to_index.size());

        return &new_rod;
    }

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

    XPBDObjects_Container _objects;
    XPBDObjectGroups_Container _object_groups;

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

    Config::SimulationConfig _config;
};

} // namespace Simulation
