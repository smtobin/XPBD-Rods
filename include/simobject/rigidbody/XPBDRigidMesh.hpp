#pragma once

#include "simobject/rigidbody/XPBDRigidBody_Base.hpp"
#include "config/XPBDRigidMeshConfig.hpp"
#include "collision/sdf/MeshSDF.hpp"

#include "common/Mesh.hpp"

namespace SimObject
{

class XPBDRigidMesh : public XPBDRigidBody_Base
{
public:
    XPBDRigidMesh(const Config::XPBDRigidMeshConfig& config);

    virtual AABB boundingBox() const override;
    const Mesh& mesh() const { return _mesh; }
    const Collision::SDF& sdf() const { return _sdf; }

private:
    Mesh _mesh;
    Vec3r _unoriented_size;

    Collision::MeshSDF _sdf;
};

} // namespace SimObject