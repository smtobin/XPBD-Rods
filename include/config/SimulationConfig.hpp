#ifndef __SIMULATION_CONFIG_HPP
#define __SIMULATION_CONFIG_HPP

#include "config/Config.hpp"
#include "config/RodConfig.hpp"

namespace Config
{

class SimulationConfig : public Config_Base
{
    public:
    explicit SimulationConfig()
        : Config_Base(), _render_config()
    {
    }

    explicit SimulationConfig(const YAML::Node& node)
        : Config_Base(node), _render_config(node)
    {
        _extractParameter("time-step", node, _time_step);
        _extractParameter("end-time", node, _end_time);
        _extractParameter("g-accel", node, _g_accel);

        for (const auto& rod_yaml_node : node["rods"])
        {
            _rod_configs.emplace_back(rod_yaml_node);
        }
    }

    explicit SimulationConfig(const std::string& name, Real time_step, Real end_time, Real g_accel)
        : Config_Base(name), _render_config()
    {
        _time_step.value = time_step;
        _end_time.value = end_time;
        _g_accel.value = g_accel;
    }

    Real timeStep() const { return _time_step.value; }
    Real endTime() const { return _end_time.value; }
    Real gAccel() const { return _g_accel.value; }

    const std::vector<RodConfig>& rodConfigs() const { return _rod_configs; }

    const SimulationRenderConfig& renderConfig() const { return _render_config; }

    protected:
    ConfigParameter<Real> _time_step = ConfigParameter<Real>(1e-3);
    ConfigParameter<Real> _end_time = ConfigParameter<Real>(60);
    ConfigParameter<Real> _g_accel = ConfigParameter<Real>(9.81);

    std::vector<RodConfig> _rod_configs;

    SimulationRenderConfig _render_config;
};

} // namespace Config

#endif // __SIMULATION_CONFIG_HPP