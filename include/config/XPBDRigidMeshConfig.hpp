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

        _render_mesh_configs.emplace(_render_mesh_configs.begin(), node);
    }

    std::string filename() const { return _filename.value;}

private:
    ConfigParameter<std::string> _filename = ConfigParameter<std::string>("");

    MeshRenderConfig _mesh_render_config;
};

} // namespace Config