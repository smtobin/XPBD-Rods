#ifndef __SIMULATION_RENDER_CONFIG_HPP
#define __SIMULATION_RENDER_CONFIG_HPP

#include "config/Config.hpp"

#include <optional>

namespace Config
{

class SimulationRenderConfig : public Config
{
    public:
    explicit SimulationRenderConfig()
        : Config()
    {

    }

    explicit SimulationRenderConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameter("hdr-image-filename", node, _hdr_image_filename);
        _extractParameter("create-skybox", node, _create_skybox);
        _extractParameter("exposure", node, _exposure);
    }

    const std::optional<std::string>& hdrImageFilename() const { return _hdr_image_filename.value; }
    bool createSkybox() const { return _create_skybox.value; }
    Real exposure() const { return _exposure.value; }

    protected:
    ConfigParameter<std::optional<std::string>> _hdr_image_filename = ConfigParameter<std::optional<std::string>>(std::nullopt);
    ConfigParameter<bool> _create_skybox = ConfigParameter<bool>(true);
    ConfigParameter<Real> _exposure = ConfigParameter<Real>(0.5);

};

} // namespace Config

#endif // __SIMULATION_RENDER_CONFIG_HPP