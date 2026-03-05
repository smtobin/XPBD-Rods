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

    /** Points (in the XY plane) for the cross-section of the rod */
    std::vector<Vec3r> _cross_section_points;

    /** Number of points to sample for each element */
    int _sample_points_per_element;



};

} // namespace Graphics