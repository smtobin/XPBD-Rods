#ifndef __SIMULATION_CONFIG_HPP
#define __SIMULATION_CONFIG_HPP

#include "config/Config.hpp"
#include "config/RodConfig.hpp"
#include "config/XPBDRigidSphereConfig.hpp"
#include "config/XPBDRigidBoxConfig.hpp"
#include "config/XPBDPendulumConfig.hpp"
#include "config/XPBDConcentricTubeRobotConfig.hpp"

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
        _extractParameter("logging", node, _logging);
        _extractParameter("logging-output-folder", node, _logging_output_dir);
        _extractParameter("logging-interval", node, _logging_interval);
        _extractParameter("log-residuals", node, _log_residuals);

        for (const auto& obj_node : node["objects"])
        {
            std::string type;
            try 
            {
                // extract type information
                type = obj_node["type"].as<std::string>();
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                std::cerr << "Type of object is needed!" << std::endl;
                continue;
            }

            if (type == "Rod")
            {
                _object_configs.template emplace_back<Config::RodConfig>(obj_node);
            }
            else if (type == "RigidBox")
            {
                _object_configs.template emplace_back<Config::XPBDRigidBoxConfig>(obj_node);
            }
            else if (type == "RigidSphere")
            {
                _object_configs.template emplace_back<Config::XPBDRigidSphereConfig>(obj_node);
            }
            else if (type == "Pendulum")
            {
                _object_configs.template emplace_back<Config::XPBDPendulumConfig>(obj_node);
            }
            else if (type == "CTR")
            {
                _object_configs.template emplace_back<Config::XPBDConcentricTubeRobotConfig>(obj_node);
            }
            else
            {
                std::cerr << "Unknown type of object! \"" << type << "\" is not a type of simulation object." << std::endl;
                assert(0);
            }
        }
    }

    explicit SimulationConfig(const std::string& name, Real time_step, Real end_time, Real g_accel, 
        bool logging, const std::string& logging_output_dir, Real logging_interval, bool log_residuals)
        : Config_Base(name), _render_config()
    {
        _time_step.value = time_step;
        _end_time.value = end_time;
        _g_accel.value = g_accel;
        _logging.value = logging;
        _logging_output_dir.value = logging_output_dir;
        _logging_interval.value = logging_interval;
        _log_residuals.value = log_residuals;
    }

    Real timeStep() const { return _time_step.value; }
    Real endTime() const { return _end_time.value; }
    Real gAccel() const { return _g_accel.value; }

    bool logging() const { return _logging.value; }
    std::string loggingOutputDir() const { return _logging_output_dir.value; }
    Real loggingInterval() const { return _logging_interval.value; }
    bool logResiduals() const { return _log_residuals.value; }

    const XPBDObjectConfigs_Container& objectConfigs() const { return _object_configs; }

    const SimulationRenderConfig& renderConfig() const { return _render_config; }

    protected:
    ConfigParameter<Real> _time_step = ConfigParameter<Real>(1e-3);
    ConfigParameter<Real> _end_time = ConfigParameter<Real>(60);
    ConfigParameter<Real> _g_accel = ConfigParameter<Real>(9.81);

    ConfigParameter<bool> _logging = ConfigParameter<bool>(false);
    ConfigParameter<std::string> _logging_output_dir = ConfigParameter<std::string>("../output/");
    ConfigParameter<Real> _logging_interval = ConfigParameter<Real>(1e-2);

    ConfigParameter<bool> _log_residuals = ConfigParameter<bool>(false);


    XPBDObjectConfigs_Container _object_configs;

    SimulationRenderConfig _render_config;
};

} // namespace Config

#endif // __SIMULATION_CONFIG_HPP