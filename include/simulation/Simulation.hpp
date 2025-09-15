#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include "common/common.hpp"

#include "simobject/rod/XPBDRod.hpp"
#include "graphics/RodGraphicsObject.hpp"
#include "graphics/GraphicsScene.hpp"

#include "config/SimulationConfig.hpp"
#include "config/SimulationRenderConfig.hpp"

#include <vector>
#include <deque>
#include <functional>
#include <atomic>

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


    // event handling
    virtual void notifyKeyPressed(const std::string& key);
    virtual void notifyKeyReleased(const std::string& key);
    virtual void notifyMouseMoved(double mx, double my);
    virtual void notifyLeftMouseButtonPressed();
    virtual void notifyLeftMouseButtonReleased();

    protected:

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
    Real _time_step;
    Real _end_time;
    Real _g_accel;
    int _viewer_refresh_time_ms;

    std::vector<SimObject::XPBDRod> _rods;

    std::deque<std::function<void()>> _callback_queue;

    // graphics
    Graphics::GraphicsScene _graphics_scene;

    Config::SimulationConfig _config;
};

} // namespace Simulation

#endif // __SIMULATION_HPP