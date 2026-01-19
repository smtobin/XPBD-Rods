#pragma once

#include "common/common.hpp"
#include "config/ObjectRenderConfig.hpp"

#include <vtkSmartPointer.h>
#include <vtkActor.h>

namespace Graphics
{

class GraphicsObject
{
public:

GraphicsObject(const Config::ObjectRenderConfig& /*config*/)
{
}

virtual ~GraphicsObject() = default;

virtual void update() = 0;

vtkSmartPointer<vtkActor> actor() { return _vtk_actor; }

protected:
    vtkSmartPointer<vtkActor> _vtk_actor;

};

} // namespace Graphics