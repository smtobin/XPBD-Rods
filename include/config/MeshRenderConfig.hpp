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
    }

    explicit MeshRenderConfig(
        RenderType render_type,
        std::string filename,
        std::optional<std::string> orm_texture_filename, std::optional<std::string> normals_texture_filename, std::optional<std::string> base_color_texture_filename,
        Real metallic, Real roughness, Real opacity, const Vec3r& color,
        bool smooth_normals, bool draw_faces, bool draw_edges
    )
        : ObjectRenderConfig(render_type, orm_texture_filename, normals_texture_filename, base_color_texture_filename,
                    metallic, roughness, opacity, color, smooth_normals)
    {
        _filename.value = filename;

        _draw_faces.value = draw_faces;
        _draw_edges.value = draw_edges;
    }

    std::string filename() const { return _filename.value; }

    bool drawFaces() const { return _draw_faces.value; }
    bool drawEdges() const { return _draw_edges.value; }

private:
    ConfigParameter<std::string> _filename = ConfigParameter<std::string>("");
    ConfigParameter<bool> _draw_faces = ConfigParameter<bool>(true);
    ConfigParameter<bool> _draw_edges = ConfigParameter<bool>(false);

};

} // namespace Config