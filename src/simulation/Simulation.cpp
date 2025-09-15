#include "simulation/Simulation.hpp"
#include "common/math.hpp"

#include <chrono>
#include <thread>

namespace Sim
{

Simulation::Simulation()
    : _setup(false),
      _time(0), _time_step(1e-3), _end_time(10),
      _g_accel(9.81), _viewer_refresh_time_ms(1000.0/30.0),
      _graphics_scene()
{

}

Simulation::Simulation(const Config::SimulationConfig& sim_config)
    : _setup(false), _time(0),
    _time_step(sim_config.timeStep()), _end_time(sim_config.endTime()), _g_accel(sim_config.gAccel()),
    _viewer_refresh_time_ms(1000.0/30.0),
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
    _rods.reserve(_config.rodConfigs().size()); // reserve space for the rods so that the vector does not need to re-allocate - all pointers to vector contents will remain valid
    for (const auto& rod_config : _config.rodConfigs())
    {
        SimObject::CircleCrossSection cross_section(rod_config.diameter()/2.0, 20);
        Mat3r base_rot_mat = Math::RotMatFromXYZEulerAngles(rod_config.initialBaseRotation());
        SimObject::XPBDRod rod(rod_config, cross_section);

        _rods.push_back(std::move(rod));
        _rods.back().setup();

        // add new rod to graphics scene to be visualized
        _graphics_scene.addObject(&_rods.back(), rod_config.renderConfig());
    }


    
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

void Simulation::notifyKeyPressed(const std::string& key)
{
    
}

void Simulation::notifyKeyReleased(const std::string& key)
{
    
}

void Simulation::notifyMouseMoved(double mx, double my)
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
    for (auto& rod : _rods)
    {
        rod.update(_time_step, _g_accel);
        
        const Vec3r& tip_pos = rod.nodes().back().position;
        const Mat3r& tip_or = rod.nodes().back().orientation;
        // std::cout << "Tip position: " << tip_pos[0] << ", " << tip_pos[1] << ", " << tip_pos[2] << std::endl;
        // std::cout << "Tip orientation:\n" << tip_or << std::endl;
    }

    _time += _time_step;
}

void Simulation::_updateGraphics()
{
    _graphics_scene.update();
}

} // namespace Simulation