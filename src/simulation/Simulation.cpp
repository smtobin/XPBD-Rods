#include "simulation/Simulation.hpp"
#include "common/math.hpp"

#include <chrono>
#include <thread>

namespace Sim
{

Simulation::Simulation()
    : _setup(false),
      _time(0), _dt(1e-3), _end_time(10),
      _g_accel(9.81), _viewer_refresh_time_ms(1000.0/30.0),
      _solver(_dt, 1),
      _graphics_scene()
{

}

Simulation::Simulation(const Config::SimulationConfig& sim_config)
    : _setup(false), _time(0),
    _dt(sim_config.timeStep()), _end_time(sim_config.endTime()), _g_accel(sim_config.gAccel()),
    _viewer_refresh_time_ms(1000.0/30.0),
    _solver(_dt, 1),
    _graphics_scene(sim_config.renderConfig()),
    _config(sim_config)
{

}


void Simulation::setup()
{
    _setup = true;

    // setup the graphics scene
    _graphics_scene.setup(this);

    // create rod(s)
    const XPBDObjectConfigs_Container& obj_configs = _config.objectConfigs();
    obj_configs.for_each_element([&](const auto& obj_config){
        _addObjectFromConfig(obj_config); 
    });

    // add constraints from object groups to the solver
    // _object_groups.for_each_element([&](const auto& obj) {
    //     const XPBDConstraints_Container& constraints = obj.constraints();
    //     constraints.for_each_element([&](const auto& constraint) {
    //         // using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;
    //         _solver.addConstraint(&constraint);
    //     });
    // });

    
}

void Simulation::update()
{
    auto wall_time_start = std::chrono::steady_clock::now();
    auto last_redraw = std::chrono::steady_clock::now();

    while (_time < _end_time)
    {
        // run any callbacks that have been queued
        for (; !_callback_queue.empty(); _callback_queue.pop_front())
        {
            _callback_queue.front()();
        }
        
        // the elapsed seconds in wall time since the simulation has started
        Real wall_time_elapsed_s = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - wall_time_start).count() / 1000000000.0;
        
        // if the simulation is ahead of the current elapsed wall time, stall
        if (_time > wall_time_elapsed_s)
        {
            continue;
        }

        _timeStep();

        // the time in ms since the viewer was last redrawn
        auto time_since_last_redraw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_redraw).count();
        // we want ~30 fps, so update the viewer every 33 ms
        if (time_since_last_redraw_ms > _viewer_refresh_time_ms)
        {
            // std::cout << _haptic_device_manager->getPosition() << std::endl;
            _updateGraphics();

            last_redraw = std::chrono::steady_clock::now();
        }
    }

    auto wall_time_end = std::chrono::steady_clock::now();
    std::cout << "Simulation " << _end_time << " seconds took " << std::chrono::duration_cast<std::chrono::milliseconds>(wall_time_end - wall_time_start).count() << " ms" << std::endl;
}

int Simulation::run()
{
    // setup if we haven't already
    if (!_setup)
        setup();
    

    // start update thread
    _graphics_scene.displayWindow();
    std::thread update_thread(&Simulation::update, this);
    
    _graphics_scene.interactorStart();
}

VecXr Simulation::primaryResidual() const
{
    // primary residual = M (x - x_tilde) + delC^T * lambda
    
    // first, create the global inertia matrix
    VecXr inertia_mat(_particle_ptr_to_index.size()*6);
    for (const auto& [particle_ptr,index] : _particle_ptr_to_index)
    {
        inertia_mat( Eigen::seqN(6*index, 3) ) = particle_ptr->mass*Vec3r::Ones();
        inertia_mat( Eigen::seqN(6*index+3, 3) ) = particle_ptr->Ib;
    }

    // then, compute (x - x_tilde)
    VecXr x_min_x_tilde(_particle_ptr_to_index.size()*6);
    for (const auto& [particle_ptr,index] : _particle_ptr_to_index)
    {
        x_min_x_tilde( Eigen::seqN(6*index, 3) ) = particle_ptr->position - _p_tilde[index];
        x_min_x_tilde( Eigen::seqN(6*index+3, 3) ) = Math::Minus_SO3(particle_ptr->orientation, _R_tilde[index] );
    }

    // now, compute delC^T * lambda
    VecXr delCT_lambda = VecXr::Zero(_particle_ptr_to_index.size()*6);
    // this requires iterating through all the constraints
    // right now, constraints are located in two places:
    //  - as internal constraints in individual objects and object groups (e.g. elastic constraints in rods)
    //  - constraints in the solver (this includes non-internal constraints for object groups)

    // iterate through internal constraints in individual objects and object groups
    auto process_internal_constraints = [&](const auto& obj) {
        std::vector<SimObject::ConstraintAndLambda> constraints_and_lambdas = obj.internalConstraintsAndLambdas();
        // iterate through the internal constraints
        for (const auto& constraint_and_lambda : constraints_and_lambdas)
        {
            // the constraint is a std::variant type, so we must use std::visit
            std::visit([&](const auto& constraint) {
                using ConstraintType = std::remove_cv_t<std::remove_pointer_t<std::decay_t<decltype(constraint)>>>;

                // iterate through the particles involved in the constraint
                const typename ConstraintType::ParticlePtrArray& particles = constraint->particles();
                for (const auto& particle : particles)
                {
                    // compute delC^T * lambda for the constraint gradient w.r.t. this particle
                    typename ConstraintType::SingleParticleGradientMatType grad = constraint->singleParticleGradient(particle);
                    Vec6r vec = grad.transpose() * Eigen::Map<const typename ConstraintType::ConstraintVecType>(constraint_and_lambda.lambda);

                    // add its contribution to the global delC^T * lambda vector
                    int index = _particle_ptr_to_index.at(particle);
                    delCT_lambda( Eigen::seqN(6*index, 6) ) += vec;
                }

            }, constraint_and_lambda.constraint);
        }
    };
    _objects.for_each_element(process_internal_constraints);
    _object_groups.for_each_element(process_internal_constraints);


    // iterate through constraints in the solver
    auto process_projector = [&](const auto& proj) {
        using ProjType = std::remove_cv_t<std::remove_reference_t<decltype(proj)>>;
        using ConstraintType = typename ProjType::Constraint;
        
        const typename ConstraintType::ParticlePtrArray& particles = proj.constraint()->particles();
        for (const auto& particle : particles)
        {
            typename ConstraintType::SingleParticleGradientMatType grad = proj.constraint()->singleParticleGradient(particle);
            Vec6r vec = grad.transpose() * proj.lambda();
            int index = _particle_ptr_to_index.at(particle);

            delCT_lambda( Eigen::seqN(6*index, 6) ) += vec;
        }
    };

    const XPBDConstraintProjectors_Container& solver_projs = _solver.constraintProjectors();
    const XPBDSeparateConstraintProjectors_Container& solver_sep_projs = _solver.separateConstraintProjectors();
    solver_projs.for_each_element(process_projector);
    solver_sep_projs.for_each_element(process_projector);

    // finally, compute the primary residual
    VecXr primary_residual = inertia_mat.asDiagonal() * x_min_x_tilde - delCT_lambda;
    return primary_residual;
}

VecXr Simulation::constraintResidual() const
{
    std::vector<Real> constraint_residual_vec;

    // right now, constraints are located in two places:
    //  - as internal constraints in individual objects and object groups (e.g. elastic constraints in rods)
    //  - constraints in the solver (this includes non-internal constraints for object groups)

    // iterate through internal constraints in individual objects and object groups
    auto process_internal_constraints = [&](const auto& obj) {
        std::vector<SimObject::ConstraintAndLambda> constraints_and_lambdas = obj.internalConstraintsAndLambdas();
        // iterate through the internal constraints
        for (const auto& constraint_and_lambda : constraints_and_lambdas)
        {
            // the constraint is a std::variant type, so we must use std::visit
            std::visit([&](const auto& constraint) {
                using ConstraintType = std::remove_cv_t<std::remove_pointer_t<std::decay_t<decltype(constraint)>>>;

                typename ConstraintType::AlphaVecType alpha_tilde = constraint->alpha() / (_dt * _dt);
                typename ConstraintType::ConstraintVecType residual = constraint->evaluate() + alpha_tilde.asDiagonal() * Eigen::Map<const typename ConstraintType::ConstraintVecType>(constraint_and_lambda.lambda);
                
                for (int i = 0; i < ConstraintType::ConstraintDim; i++)
                {
                    constraint_residual_vec.push_back(residual[i]);
                }
                

            }, constraint_and_lambda.constraint);
        }
    };
    _objects.for_each_element(process_internal_constraints);
    _object_groups.for_each_element(process_internal_constraints);


    // iterate through constraints in the solver
    auto process_projector = [&](const auto& proj) {
        using ProjType = std::remove_cv_t<std::remove_reference_t<decltype(proj)>>;
        using ConstraintType = typename ProjType::Constraint;
        
        typename ConstraintType::AlphaVecType alpha_tilde = proj.constraint()->alpha() / (_dt * _dt);
        typename ConstraintType::ConstraintVecType residual = proj.constraint()->evaluate() + alpha_tilde.asDiagonal() * proj.lambda();
        
        for (int i = 0; i < ConstraintType::ConstraintDim; i++)
        {
            constraint_residual_vec.push_back(residual[i]);
        }
    };

    const XPBDConstraintProjectors_Container& solver_projs = _solver.constraintProjectors();
    const XPBDSeparateConstraintProjectors_Container& solver_sep_projs = _solver.separateConstraintProjectors();
    solver_projs.for_each_element(process_projector);
    solver_sep_projs.for_each_element(process_projector);

    VecXr constraint_residual = Eigen::Map<VecXr>(constraint_residual_vec.data(), constraint_residual_vec.size());
    return constraint_residual;
}

void Simulation::notifyKeyPressed(const std::string& /*key*/)
{
    
}

void Simulation::notifyKeyReleased(const std::string& /*key*/)
{
    
}

void Simulation::notifyMouseMoved(double /*mx*/, double /*my*/)
{
    
}

void Simulation::notifyLeftMouseButtonPressed()
{

}

void Simulation::notifyLeftMouseButtonReleased()
{

}

void Simulation::_timeStep()
{
    // std::cout << "t=" << _time << std::endl;
    _objects.for_each_element([&](auto& obj) { obj.inertialUpdate(_dt); });
    _object_groups.for_each_element([&](auto& obj) { obj.inertialUpdate(_dt); });

    // store the inertially predicted positions and orientations
    // useful for computing the primary residual
    for (const auto& [particle_ptr, index] : _particle_ptr_to_index)
    {
        _p_tilde[index] = particle_ptr->position;
        _R_tilde[index] = particle_ptr->orientation;
    }


    _objects.for_each_element([&](auto& obj) { obj.internalConstraintSolve(_dt); });
    _object_groups.for_each_element([&](auto& obj) { obj.internalConstraintSolve(_dt); });

    // TODO: solve constraints here
    _solver.solve();

    _objects.for_each_element([&](auto& obj) { obj.velocityUpdate(_dt); });
    _object_groups.for_each_element([&](auto& obj) { obj.velocityUpdate(_dt); });

    _time += _dt;
}

void Simulation::_updateGraphics()
{
    _graphics_scene.update();
}

} // namespace Simulation