#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include "common/common.hpp"

#include "rod/XPBDRod.hpp"
#include "graphics/RodGraphicsObject.hpp"
#include "graphics/GraphicsScene.hpp"

#include "config/SimulationConfig.hpp"
#include "config/SimulationRenderConfig.hpp"

#include <vector>
#include <atomic>

namespace Sim
{

class Simulation
{
    public:
    explicit Simulation();

    explicit Simulation(const Config::SimulationConfig& sim_config, const Config::SimulationRenderConfig& sim_render_config);

    void setup();

    int run();

    void update();

    private:

    void _timeStep();

    void _updateGraphics();

    private:
    bool _setup;

    Real _time;
    Real _time_step;
    Real _end_time;
    Real _g_accel;
    int _viewer_refresh_time_ms;

    std::vector<Rod::XPBDRod> _rods;

    // graphics
    Graphics::GraphicsScene _graphics_scene;

    Config::SimulationConfig _config;
};

} // namespace Simulation

#endif // __SIMULATION_HPP