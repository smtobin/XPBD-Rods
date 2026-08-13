#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidMeshConfig.hpp"

#include "common/Mesh.hpp"

namespace SimObject
{

class XPBDRigidMesh : public XPBDRigidBody_Base
{
public:
    XPBDRigidMesh(const Config::XPBDRigidMeshConfig& config);

    virtual AABB boundingBox() const override;
    const Mesh& mesh() const { return _mesh; }

private:
    Mesh _mesh;
};

} // namespace SimObject