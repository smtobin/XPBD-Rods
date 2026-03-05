#include "graphics/HigherOrderRodGraphicsObject.hpp"
#include "graphics/VTKUtils.hpp"

#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>

#include <vtkTexture.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataTangents.h>
#include <vtkPolyDataMapper.h>
#include <vtkPNGReader.h>
#include <vtkCleanPolyData.h>
#include <vtkImageData.h>

#include <filesystem>
#include <optional>

namespace Graphics
{

template<int Order>
HigherOrderRodGraphicsObject<Order>::HigherOrderRodGraphicsObject(const SimObject::XPBDRod_<Order>* rod, const Config::ObjectRenderConfig& render_config)
    : GraphicsObject(render_config), _rod(rod), _render_config(render_config)
{
    _vtk_poly_data = vtkSmartPointer<vtkPolyData>::New();
    _vtk_poly_data_normals = vtkSmartPointer<vtkPolyDataNormals>::New();
    _vtk_poly_data_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

    _generateInitialPolyData();

    vtkNew<vtkPolyDataMapper> data_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputData(_vtk_poly_data);
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        data_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        data_mapper->SetInputData(_vtk_poly_data);
    }

    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(data_mapper);

    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

    // Add this step for smooth normals
    // vtkNew<vtkPolyDataNormals> normalGenerator;
    // normalGenerator->SetInputData(_vtk_poly_data);
    // normalGenerator->SetFeatureAngle(30.0); // Force smooth normals
    // normalGenerator->SplittingOff();
    // normalGenerator->ConsistencyOn();
    // normalGenerator->AutoOrientNormalsOn();
    // normalGenerator->ComputePointNormalsOn();
    // normalGenerator->ComputeCellNormalsOff();
    // normalGenerator->FlipNormalsOff();
    // normalGenerator->Update();

    // vtkNew<vtkPolyDataTangents> tangents;
    // tangents->SetInputConnection(normalGenerator->GetOutputPort());
    // tangents->Update();

    // vtkNew<vtkPolyDataMapper> mapper;
    // mapper->SetInputConnection(tangents->GetOutputPort());
    // _vtk_actor->SetMapper(mapper);

}

template<int Order>
void HigherOrderRodGraphicsObject<Order>::update()
{
    _updatePolyData();

}

template<int Order>
void HigherOrderRodGraphicsObject<Order>::_generateInitialPolyData()
{
    const std::vector<SimObject::RodElement<Order>>& elements = _rod->elements();
    _sample_points_per_element = std::max(1, static_cast<int>(Order * elements[0].restLength() / _rod->radius()));

    const std::vector<SimObject::OrientedParticle>& nodes = _rod->nodes();

    // create cross-section points for a circular cross-section
    int tubular_resolution = 20;
    _cross_section_points.resize(tubular_resolution);
    for (int i = 0; i < tubular_resolution; i++)
    {
        const Real angle = i * 2 * M_PI / tubular_resolution;
        _cross_section_points[i] = Vec3r( _rod->radius() * std::cos(angle), _rod->radius() * std::sin(angle), 0);
    }

    // insert cross-section vertices
    vtkNew<vtkPoints> points;
    for (unsigned ei = 0; ei < elements.size(); ei++)
    {
        int num_samples = _sample_points_per_element;
        if (ei == elements.size()-1)
            num_samples = _sample_points_per_element + 1;

        for (int si = 0; si < num_samples; si++)
        {
            Real s_hat = si * 1.0 / _sample_points_per_element;

            Vec3r position = elements[ei].position(s_hat);
            Mat3r orientation = elements[ei].orientation(s_hat);
            // for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
            for (const auto& p : _cross_section_points)
            {
                // transform each point in local cross section to global frame
                const Vec3r transformed = orientation * p + position;
                points->InsertNextPoint( transformed[0], transformed[1], transformed[2] );
            }
        }
        
    }

    // create faces inbetween nodes
    int cs_ind = 0;
    vtkNew<vtkCellArray> faces;
    for (unsigned ei = 0; ei < elements.size(); ei++)
    {
        int num_samples = _sample_points_per_element;

        for (int si = 0; si < num_samples; si++)
        {

            for (unsigned pi = 0; pi < _cross_section_points.size(); pi++)
            {
                // vertices for the quad face
                const int v1 = cs_ind*_cross_section_points.size() + pi;
                const int v2 = (pi != _cross_section_points.size()-1) ? v1 + 1 : cs_ind*_cross_section_points.size();
                const int v3 = v2 + _cross_section_points.size();
                const int v4 = v1 + _cross_section_points.size();

                vtkNew<vtkTriangle> tri1;
                tri1->GetPointIds()->SetId(0, v1);
                tri1->GetPointIds()->SetId(1, v2);
                tri1->GetPointIds()->SetId(2, v3);

                vtkNew<vtkTriangle> tri2;
                tri2->GetPointIds()->SetId(0, v1);
                tri2->GetPointIds()->SetId(1, v3);
                tri2->GetPointIds()->SetId(2, v4);
                faces->InsertNextCell(tri1);
                faces->InsertNextCell(tri2);
            }

            cs_ind++;
        }
        
    }

    int num_cs = cs_ind+1;

    // end cap points
    points->InsertNextPoint( nodes[0].position[0], nodes[0].position[1], nodes[0].position[2] );
    points->InsertNextPoint( nodes.back().position[0], nodes.back().position[1], nodes.back().position[2] );

    // end cap faces
    // base faces
    for (unsigned pi = 0; pi < _cross_section_points.size(); pi++)
    {
        const int v1 = pi;
        const int v2 = num_cs*_cross_section_points.size();
        const int v3 = (pi != _cross_section_points.size()-1) ? v1 + 1 : 0;

        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, v1);
        tri->GetPointIds()->SetId(1, v2);
        tri->GetPointIds()->SetId(2, v3);
        faces->InsertNextCell(tri);
    }

    // tip faces
    for (unsigned pi = 0; pi < _cross_section_points.size(); pi++)
    {
        const int v1 = (num_cs-1)*_cross_section_points.size() + pi;
        const int v2 = (pi != _cross_section_points.size()-1) ? v1 + 1 : (num_cs-1)*_cross_section_points.size();
        const int v3 = num_cs*_cross_section_points.size() + 1;
        

        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, v1);
        tri->GetPointIds()->SetId(1, v2);
        tri->GetPointIds()->SetId(2, v3);
        faces->InsertNextCell(tri);
    }

    // vtkNew<vtkPolygon> base_face, end_face;
    // base_face->GetPointIds()->SetNumberOfIds(cross_section_points.size());
    // end_face->GetPointIds()->SetNumberOfIds(cross_section_points.size());
    // for (unsigned i = 0; i < cross_section_points.size(); i++)
    // {
    //     base_face->GetPointIds()->SetId(i, i);
    //     end_face->GetPointIds()->SetId(i, (nodes.size()-1)*cross_section_points.size() + i);
    // }
    // faces->InsertNextCell(base_face);
    // faces->InsertNextCell(end_face);

    // vtkNew<vtkPolyData> rod_poly_data;
    _vtk_poly_data->SetPoints(points);
    _vtk_poly_data->SetPolys(faces);

    // Create and set texture coordinates
    // vtkNew<vtkFloatArray> textureCoords;
    // textureCoords->SetNumberOfComponents(2);
    // textureCoords->SetNumberOfTuples(nodes.size()*_cross_section_points.size());
    // textureCoords->SetName("TextureCoordinates");
    // for (unsigned ni = 0; ni < nodes.size(); ni++)
    // {
    //     float x_coord = (ni % 2 == 0) ? 0.0f : 0.5f;
    //     for (unsigned pi = 0; pi < _cross_section_points.size(); pi++)
    //     {
    //         float y_coord = static_cast<float>(pi) / _cross_section_points.size();
    //         textureCoords->SetTuple2(ni*_cross_section_points.size() + pi, x_coord, y_coord);
    //     }
    // }
    // _vtk_poly_data->GetPointData()->SetTCoords(textureCoords);
}

template<int Order>
void HigherOrderRodGraphicsObject<Order>::_updatePolyData()
{
    // TODO: use vtkPoints::setData and vtkFloatArray? supposed to be faster

    const std::vector<SimObject::OrientedParticle>& nodes = _rod->nodes();
    const std::vector<SimObject::RodElement<Order>>& elements = _rod->elements();

    vtkPoints* points = _vtk_poly_data->GetPoints();
    int cs_ind = 0;
    for (unsigned ei = 0; ei < elements.size(); ei++)
    {
        int num_samples = _sample_points_per_element;
        if (ei == elements.size()-1)
            num_samples = _sample_points_per_element + 1;

        for (int si = 0; si < num_samples; si++)
        {
            Real s_hat = si * 1.0 / _sample_points_per_element;

            Vec3r position = elements[ei].position(s_hat);
            Mat3r orientation = elements[ei].orientation(s_hat);
            // for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
            for (unsigned pi = 0; pi < _cross_section_points.size(); pi++)
            {
                // transform each point in local cross section to global frame
                const Vec3r transformed = orientation * _cross_section_points[pi] + position;
                points->SetPoint( cs_ind*_cross_section_points.size() + pi, transformed.data() );
            }

            cs_ind++;
        }
        
    }

    points->SetPoint(cs_ind*_cross_section_points.size(), nodes[0].position[0], nodes[0].position[1], nodes[0].position[2]);
    points->SetPoint(cs_ind*_cross_section_points.size()+1, nodes.back().position[0], nodes.back().position[1], nodes.back().position[2]);
    points->Modified();
}

template class HigherOrderRodGraphicsObject<1>;
template class HigherOrderRodGraphicsObject<2>;

} // namespace Graphics