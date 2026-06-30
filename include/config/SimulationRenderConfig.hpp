#pragma once

#include "config/Config.hpp"

#include <optional>

namespace Config
{

class SimulationRenderConfig : public Config_Base
{
    public:
    explicit SimulationRenderConfig()
        : Config_Base()
    {
        std::cout << "Default constructor" << std::endl;
        assert(0);
    }

    explicit SimulationRenderConfig(const YAML::Node& node)
        : Config_Base(node)
    {
        _extractParameter("hdr-image-filename", node, _hdr_image_filename);
        _extractParameter("create-skybox", node, _create_skybox);
        _extractParameter("hdr-scaling", node, _hdr_scaling);

        _extractParameter("camera-position", node, _camera_position);
        _extractParameter("camera-focal-point", node, _camera_focal_point);
        _extractParameter("camera-orthographic", node, _camera_orthographic);
    }

    const std::optional<std::string>& hdrImageFilename() const { return _hdr_image_filename.value; }
    bool createSkybox() const { return _create_skybox.value; }
    Real hdrScaling() const { return _hdr_scaling.value; }
    Vec3r cameraPosition() const { return _camera_position.value; }
    Vec3r cameraFocalPoint() const { return _camera_focal_point.value; }
    bool cameraOrthographic() const { return _camera_orthographic.value; }


    protected:
    ConfigParameter<std::optional<std::string>> _hdr_image_filename = ConfigParameter<std::optional<std::string>>();
    ConfigParameter<bool> _create_skybox = ConfigParameter<bool>(true);
    ConfigParameter<Real> _hdr_scaling = ConfigParameter<Real>(0.2);

    ConfigParameter<Vec3r> _camera_position = ConfigParameter<Vec3r>(Vec3r(3, 4, 3));
    ConfigParameter<Vec3r> _camera_focal_point = ConfigParameter<Vec3r>(Vec3r(0, 0, 0));
    ConfigParameter<bool> _camera_orthographic = ConfigParameter<bool>(false);

};

} // namespace Config