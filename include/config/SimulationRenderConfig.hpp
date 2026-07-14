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
        _extractParameter("window-width", node, _window_width);
        _extractParameter("window-height", node, _window_height);

        _extractParameter("hdr-image-filename", node, _hdr_image_filename);
        _extractParameter("create-skybox", node, _create_skybox);
        _extractParameter("hdr-scaling", node, _hdr_scaling);

        _extractParameter("camera-position", node, _camera_position);
        _extractParameter("camera-focal-point", node, _camera_focal_point);
        _extractParameter("camera-orthographic", node, _camera_orthographic);

        _extractParameter("render-for-video", node, _render_for_video);
        _extractParameter("render-output-folder", node, _render_output_folder);
    }

    Real windowWidth() const { return _window_width.value; }
    Real windowHeight() const { return _window_height.value; }
    const std::optional<std::string>& hdrImageFilename() const { return _hdr_image_filename.value; }
    bool createSkybox() const { return _create_skybox.value; }
    Real hdrScaling() const { return _hdr_scaling.value; }
    Vec3r cameraPosition() const { return _camera_position.value; }
    Vec3r cameraFocalPoint() const { return _camera_focal_point.value; }
    bool cameraOrthographic() const { return _camera_orthographic.value; }
    bool renderForVideo() const { return _render_for_video.value; }
    std::string renderOutputFolder() const { return _render_output_folder.value; }


    protected:
    ConfigParameter<int> _window_width = ConfigParameter<int>(600);
    ConfigParameter<int> _window_height = ConfigParameter<int>(600);

    ConfigParameter<std::optional<std::string>> _hdr_image_filename = ConfigParameter<std::optional<std::string>>();
    ConfigParameter<bool> _create_skybox = ConfigParameter<bool>(true);
    ConfigParameter<Real> _hdr_scaling = ConfigParameter<Real>(0.2);

    ConfigParameter<Vec3r> _camera_position = ConfigParameter<Vec3r>(Vec3r(3, 4, 3));
    ConfigParameter<Vec3r> _camera_focal_point = ConfigParameter<Vec3r>(Vec3r(0, 0, 0));
    ConfigParameter<bool> _camera_orthographic = ConfigParameter<bool>(false);

    ConfigParameter<bool> _render_for_video = ConfigParameter<bool>(false);
    ConfigParameter<std::string> _render_output_folder = ConfigParameter<std::string>(".");

};

} // namespace Config