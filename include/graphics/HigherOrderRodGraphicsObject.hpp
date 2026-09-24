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

template <typename ElementType>
class HigherOrderRodGraphicsObject : public GraphicsObject
{
public:
    explicit HigherOrderRodGraphicsObject(const SimObject::XPBDRod_<ElementType>* rod, const Config::ObjectRenderConfig& render_config);

    virtual void update() override;

    vtkSmartPointer<vtkActor> centerlineActor() { return _vtk_centerline_actor; }
    vtkSmartPointer<vtkActor> framesActor() { return _frames_actor; }

private:
    void _generateInitialPolyData();
    void _updatePolyData();

    private:
    const SimObject::XPBDRod_<ElementType>* _rod;

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;
    vtkSmartPointer<vtkPolyDataNormals> _vtk_poly_data_normals;
    vtkSmartPointer<vtkPolyDataMapper> _vtk_poly_data_mapper;

    vtkSmartPointer<vtkActor> _vtk_centerline_actor;
    vtkSmartPointer<vtkPolyData> _vtk_centerline_poly_data;

    Config::ObjectRenderConfig _render_config;

    /** Points (in the XY plane) for the cross-section of the rod */
    std::vector<Vec3r> _cross_section_points;

    /** Whether or not to color each rod element individually.
     * When enabled, a fixed number of samples per element will be used.
     */
    bool _color_elements;

    /** Number of points to sample for each element */
    int _sample_points_per_element = 3;

    /** Number of cross-sections to use along the rod. */
    int _num_samples;

    /** Whether or not to visualize the centerline of the rod. */
    bool _draw_centerline;

/** Whether or not to draw end caps on the rod. */
    bool _draw_end_caps;

    /** Whether or not to draw frames at nodes in the rod */
    bool _draw_frames;
    Real _frame_scale = 0.1;
    vtkSmartPointer<vtkPolyData> _frames_poly_data;
    vtkSmartPointer<vtkActor> _frames_actor;

    /** Number of cross sections in the end caps (when applicable) */
    int _cap_resolution = 6;

    /** Number of points in each circular cross section */
    int _tubular_resolution = 20;



};

} // namespace Graphics