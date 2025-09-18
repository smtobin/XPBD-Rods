#ifndef __OBJECT_RENDER_CONFIG_HPP
#define __OBJECT_RENDER_CONFIG_HPP

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
        _extractParameterWithOptions("render-type", node, _render_type, RENDER_TYPE_MAP());

        _extractParameter("orm-texture-filename", node, _orm_texture_filename);
        _extractParameter("normals-texture-filename", node, _normals_texture_filename);
        _extractParameter("base-color-texture-filename", node, _base_color_texture_filename);

        _extractParameter("metallic", node, _metallic);
        _extractParameter("roughness", node, _roughness);
        _extractParameter("color", node, _color);
    }

    RenderType renderType() const { return _render_type.value; }
    std::optional<std::string> ormTextureFilename() const { return _orm_texture_filename.value; }
    std::optional<std::string> normalsTextureFilename() const { return _normals_texture_filename.value; }
    std::optional<std::string> baseColorTextureFilename() const { return _base_color_texture_filename.value; }

    Real metallic() const { return _metallic.value; }
    Real roughness() const { return _roughness.value; }
    const Vec3r& color() const { return _color.value; }

    protected:
    ConfigParameter<RenderType> _render_type = ConfigParameter<RenderType>(RenderType::PHONG); 

    // PBR texture filenames
    ConfigParameter<std::optional<std::string>> _orm_texture_filename = ConfigParameter<std::optional<std::string>>(std::nullopt);
    ConfigParameter<std::optional<std::string>> _normals_texture_filename = ConfigParameter<std::optional<std::string>>(std::nullopt);
    ConfigParameter<std::optional<std::string>> _base_color_texture_filename = ConfigParameter<std::optional<std::string>>(std::nullopt);

    ConfigParameter<Real> _metallic = ConfigParameter<Real>(0.0);
    ConfigParameter<Real> _roughness = ConfigParameter<Real>(0.5);
    ConfigParameter<Vec3r> _color = ConfigParameter<Vec3r>(Vec3r(0.8, 0.8, 0.8));
};

} // namespace Config

#endif // __OBJECT_RENDER_CONFIG_HPP