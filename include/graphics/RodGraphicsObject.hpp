#ifndef __ROD_GRAPHICS_OBJECT_HPP
#define __ROD_GRAPHICS_OBJECT_HPP

#include "rod/XPBDRod.hpp"
#include "config/ObjectRenderConfig.hpp"

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>

#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>

namespace Graphics
{

class RodGraphicsObject
{

    public:
    explicit RodGraphicsObject(const Rod::XPBDRod* rod, const Config::ObjectRenderConfig* render_config);

    void setup();
    void update();

    vtkSmartPointer<vtkActor> getVtkActor() const { return _vtk_actor; }

    private:
    void _generateInitialPolyData();
    void _updatePolyData();

    private:
    const Rod::XPBDRod* _rod;

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;
    vtkSmartPointer<vtkPolyDataNormals> _vtk_poly_data_normals;
    vtkSmartPointer<vtkPolyDataMapper> _vtk_poly_data_mapper;
    vtkSmartPointer<vtkActor> _vtk_actor;

    const Config::ObjectRenderConfig* _render_config;



};

} // namespace Graphics

#endif // __ROD_GRAPHICS_OBJECT_HPP