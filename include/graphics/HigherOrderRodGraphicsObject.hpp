#pragma once

#include "simobject/rod/XPBDHigherOrderRod.hpp"
#include "graphics/GraphicsObject.hpp"
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

template <int Order>
class HigherOrderRodGraphicsObject : public GraphicsObject
{
public:
    explicit HigherOrderRodGraphicsObject(const SimObject::XPBDRod_<Order>* rod, const Config::ObjectRenderConfig& render_config);

    virtual void update() override;

private:
    void _generateInitialPolyData();
    void _updatePolyData();

    private:
    const SimObject::XPBDRod_<Order>* _rod;

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;
    vtkSmartPointer<vtkPolyDataNormals> _vtk_poly_data_normals;
    vtkSmartPointer<vtkPolyDataMapper> _vtk_poly_data_mapper;

    Config::ObjectRenderConfig _render_config;



};

} // namespace Graphics