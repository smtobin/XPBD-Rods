#pragma once

#include "simobject/rigidbody/XPBDPlane.hpp"
#include "config/ObjectRenderConfig.hpp"
#include "graphics/GraphicsObject.hpp"

#include <vtkPlaneSource.h>
#include <vtkActor.h>
#include <vtkTransform.h>

namespace Graphics
{

class PlaneGraphicsObject : public GraphicsObject
{
public:
    explicit PlaneGraphicsObject(const SimObject::XPBDPlane* sphere, const Config::ObjectRenderConfig& render_config);

    virtual void update() override;

    const SimObject::XPBDPlane* plane() const { return _plane; }

    private:
    const SimObject::XPBDPlane* _plane;

    vtkSmartPointer<vtkPlaneSource> _plane_source;    // sphere source algorithm that generates box mesh (keep a reference to it so we can potentially change it's size)
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of sphere)
};

} // namespace Graphics