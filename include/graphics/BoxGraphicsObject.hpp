#pragma once

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "config/ObjectRenderConfig.hpp"
#include "graphics/GraphicsObject.hpp"

#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>

namespace Graphics
{

class BoxGraphicsObject : public GraphicsObject
{
    public:
    explicit BoxGraphicsObject(const SimObject::XPBDRigidBox* box, const Config::ObjectRenderConfig& render_config);

    virtual void update() override;

    const SimObject::XPBDRigidBox* box() const { return _box; }


    private:
    const SimObject::XPBDRigidBox* _box;

    vtkSmartPointer<vtkCubeSource> _cube_source;    // cube source algorithm that generates box mesh (keep a reference to it so we can potentially change it's size)
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of box)
};

} // namespace Graphics