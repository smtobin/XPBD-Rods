#pragma once

#include "config/Config.hpp"

#include <string>
#include <map>
#include <optional>

namespace Config
{

class ObjectRenderConfig : public Config_Base
{
    public:
    enum class RenderType
    {
        PBR=0,
        PHONG,
        FLAT
    };

    static std::map<std::string, RenderType> RENDER_TYPE_MAP()
    {
        static std::map<std::string, RenderType> render_map{
            {"PBR", RenderType::PBR},
            {"Phong", RenderType::PHONG},
            {"Flat", RenderType::FLAT}
        };

        return render_map;
    }

    public:
    explicit ObjectRenderConfig()
        : Config_Base()
    {

    }

    explicit ObjectRenderConfig(const YAML::Node& node)
        : Config_Base(node)
    {
        _extractParameter("render", node, _render);
        _extractParameterWithOptions("render-type", node, _render_type, RENDER_TYPE_MAP());

        _extractParameter("orm-texture-filename", node, _orm_texture_filename);
        _extractParameter("normals-texture-filename", node, _normals_texture_filename);
        _extractParameter("base-color-texture-filename", node, _base_color_texture_filename);

        _extractParameter("metallic", node, _metallic);
        _extractParameter("roughness", node, _roughness);
        _extractParameter("opacity", node, _opacity);
        _extractParameter("color", node, _color);

        _extractParameter("smooth-normals", node, _smooth_normals);

        _extractParameter("draw-centerline", node, _draw_centerline);
        _extractParameter("centerline-samples", node, _num_centerline_samples);
        _extractParameter("color-elements", node, _color_elements);
        _extractParameter("draw-end-caps", node, _draw_end_caps);
        _extractParameter("draw-frames", node, _draw_frames);
    }

    explicit ObjectRenderConfig(
        RenderType render_type,
        std::optional<std::string> orm_texture_filename, std::optional<std::string> normals_texture_filename, std::optional<std::string> base_color_texture_filename,
        Real metallic, Real roughness, Real opacity, const Vec3r& color,
        bool smooth_normals
    )
    {
        _render_type.value = render_type;

        _orm_texture_filename.value = orm_texture_filename;
        _normals_texture_filename.value = normals_texture_filename;
        _base_color_texture_filename.value = base_color_texture_filename;

        _metallic.value = metallic;
        _roughness.value = roughness;
        _opacity.value = opacity;
        _color.value = color;

        _smooth_normals.value = smooth_normals;
    }

    bool render() const { return _render.value; }
    RenderType renderType() const { return _render_type.value; }
    std::optional<std::string> ormTextureFilename() const { return _orm_texture_filename.value; }
    std::optional<std::string> normalsTextureFilename() const { return _normals_texture_filename.value; }
    std::optional<std::string> baseColorTextureFilename() const { return _base_color_texture_filename.value; }

    Real metallic() const { return _metallic.value; }
    Real roughness() const { return _roughness.value; }
    Real opacity() const { return _opacity.value; }
    Vec3r color() const { return _color.value; }

    void setRender(bool render) { _render.value = render; }
    void setMetallic(Real metallic) { _metallic.value = metallic; }
    void setRoughness(Real roughness) { _roughness.value = roughness; }
    void setOpacity(Real opacity) { _opacity.value = opacity; }
    void setColor(Vec3r color) { _color.value = color; }
    void setCenterlineSamples(int samples) { _num_centerline_samples.value = samples; }
    void setDrawEndCaps(bool draw_end_caps) { _draw_end_caps.value = draw_end_caps; }
    void setDrawFrames(bool draw_frames) { _draw_frames.value = draw_frames; }

    bool smoothNormals() const { return _smooth_normals.value; }

    bool drawCenterline() const { return _draw_centerline.value; }
    int numCenterlineSamples() const { return _num_centerline_samples.value; }
    bool colorElements() const { return _color_elements.value; }
    bool drawEndCaps() const { return _draw_end_caps.value; }
    bool drawFrames() const { return _draw_frames.value; }

    protected:
    ConfigParameter<bool> _render = ConfigParameter<bool>(true);
    ConfigParameter<RenderType> _render_type = ConfigParameter<RenderType>(RenderType::PBR); 

    // PBR texture filenames
    ConfigParameter<std::optional<std::string>> _orm_texture_filename;
    ConfigParameter<std::optional<std::string>> _normals_texture_filename;
    ConfigParameter<std::optional<std::string>> _base_color_texture_filename;

    ConfigParameter<Real> _metallic = ConfigParameter<Real>(0.0);
    ConfigParameter<Real> _roughness = ConfigParameter<Real>(0.5);
    ConfigParameter<Real> _opacity = ConfigParameter<Real>(1.0);
    ConfigParameter<Vec3r> _color = ConfigParameter<Vec3r>(Vec3r(1.0, 1.0, 1.0));

    ConfigParameter<bool> _smooth_normals = ConfigParameter<bool>(true);

    /** Rod-specific options */
    ConfigParameter<bool> _draw_centerline = ConfigParameter<bool>(false);
    ConfigParameter<int> _num_centerline_samples = ConfigParameter<int>(50);
    ConfigParameter<bool> _color_elements = ConfigParameter<bool>(false);
    ConfigParameter<bool> _draw_end_caps = ConfigParameter<bool>(false);
    ConfigParameter<bool> _draw_frames = ConfigParameter<bool>(false);
    
};

} // namespace Config