#pragma once

#include "config/ObjectRenderConfig.hpp"

namespace Config
{

class MeshRenderConfig : public ObjectRenderConfig
{
public:
    explicit MeshRenderConfig()
        : ObjectRenderConfig()
    {}

    explicit MeshRenderConfig(const YAML::Node& node)
        : ObjectRenderConfig(node)
    {
        _extractParameter("filename", node, _filename);

        _extractParameter("draw-faces", node, _draw_faces);
        _extractParameter("draw-edges", node, _draw_edges);

        _extractParameter("scale", node, _scale);
        _extractParameter("rotation", node, _rotation);
        _extractParameter("translation", node, _translation);
    }

    explicit MeshRenderConfig(
        RenderType render_type,
        std::string filename,
        std::optional<std::string> orm_texture_filename, std::optional<std::string> normals_texture_filename, std::optional<std::string> base_color_texture_filename,
        Real metallic, Real roughness, Real opacity, const Vec3r& color,
        bool smooth_normals, bool draw_faces, bool draw_edges,
        const Vec3r& translation, const Vec3r& rotation, const Vec3r& scale
    )
        : ObjectRenderConfig(render_type, orm_texture_filename, normals_texture_filename, base_color_texture_filename,
                    metallic, roughness, opacity, color, smooth_normals)
    {
        _filename.value = filename;

        _draw_faces.value = draw_faces;
        _draw_edges.value = draw_edges;

        _translation.value = translation;
        _rotation.value = rotation;
        _scale.value = scale;
    }

    std::string filename() const { return _filename.value; }

    bool drawFaces() const { return _draw_faces.value; }
    bool drawEdges() const { return _draw_edges.value; }

    Vec3r scale() const { return _scale.value; }
    Vec3r rotation() const { return _rotation.value; }
    Vec3r translation() const { return _translation.value; }

private:
    ConfigParameter<std::string> _filename = ConfigParameter<std::string>("");
    ConfigParameter<bool> _draw_faces = ConfigParameter<bool>(true);
    ConfigParameter<bool> _draw_edges = ConfigParameter<bool>(false);

    ConfigParameter<Vec3r> _scale = ConfigParameter<Vec3r>(Vec3r(1,1,1));
    ConfigParameter<Vec3r> _rotation = ConfigParameter<Vec3r>(Vec3r(0,0,0));
    ConfigParameter<Vec3r> _translation = ConfigParameter<Vec3r>(Vec3r(0,0,0));

};

} // namespace Config