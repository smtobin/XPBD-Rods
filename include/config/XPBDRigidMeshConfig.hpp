#pragma once

#include "config/XPBDRigidBodyConfig.hpp"
#include "config/MeshRenderConfig.hpp"

namespace SimObject
{
    class XPBDRigidMesh;
}

namespace Config
{

class XPBDRigidMeshConfig : public XPBDRigidBodyConfig
{
public:
    using SimObjectType = SimObject::XPBDRigidMesh;

    explicit XPBDRigidMeshConfig()
        : XPBDRigidBodyConfig()
    {}

    explicit XPBDRigidMeshConfig(const YAML::Node& node)
        : XPBDRigidBodyConfig(node), _mesh_render_config(node)
    {
        _extractParameter("filename", node, _filename);
        _extractParameter("scale", node, _scale);

        _render_mesh_configs.emplace(_render_mesh_configs.begin(), node);
    }

    XPBDRigidMeshConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, bool collisions, Real mu_s, Real mu_d,
        Real density, bool fixed,
        const std::string& filename, const Vec3r& scale)
        : XPBDRigidBodyConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, collisions, mu_s, mu_d, density, fixed)
    {
        _filename.value = filename;
        _scale.value = scale;

        _render_mesh_configs.insert(_render_mesh_configs.begin(), MeshRenderConfig());
    }

    std::string filename() const { return _filename.value;}
    Vec3r scale() const { return _scale.value; }

    MeshRenderConfig& meshRenderConfig() { return _render_mesh_configs[0]; }

private:
    ConfigParameter<std::string> _filename = ConfigParameter<std::string>("");
    ConfigParameter<Vec3r> _scale = ConfigParameter<Vec3r>(Vec3r(1,1,1));

    MeshRenderConfig _mesh_render_config;
};

} // namespace Config