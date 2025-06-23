#include "graphics/RodGraphicsObject.hpp"

#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>

namespace Graphics
{

RodGraphicsObject::RodGraphicsObject(const Rod::XPBDRod* rod)
    : _rod(rod)
{

}

void RodGraphicsObject::setup()
{
    _vtk_poly_data = vtkSmartPointer<vtkPolyData>::New();
    _vtk_poly_data_normals = vtkSmartPointer<vtkPolyDataNormals>::New();
    _vtk_poly_data_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    _vtk_actor = vtkSmartPointer<vtkActor>::New();

    _generateInitialPolyData();

    _vtk_poly_data_normals->SetInputData(_vtk_poly_data);
    _vtk_poly_data_normals->ComputePointNormalsOn();
    _vtk_poly_data_normals->ComputeCellNormalsOn();
    _vtk_poly_data_normals->SplittingOn();
    // rod_normals->FlipNormalsOn();
    _vtk_poly_data_normals->Update();

    // rod_mapper->SetInputData(rod_poly_data);
    _vtk_poly_data_mapper->SetInputConnection(_vtk_poly_data_normals->GetOutputPort());

    // create the actor for the rod
    _vtk_actor->SetMapper(_vtk_poly_data_mapper);
    // rod_actor->GetProperty()->SetInterpolationToPBR();
    _vtk_actor->GetProperty()->SetInterpolationToPhong();
    _vtk_actor->GetProperty()->SetColor(0.75, 0.75, 0.75); // Base color (albedo)
    
    // PBR Material Properties - adjusted for better visibility
    _vtk_actor->GetProperty()->SetMetallic(1.0); 
    _vtk_actor->GetProperty()->SetRoughness(0.3);       
    _vtk_actor->GetProperty()->SetAnisotropy(0.0);     
    _vtk_actor->GetProperty()->SetBaseIOR(1.5);  
    
    _vtk_actor->GetProperty()->SetOcclusionStrength(1.0);
}

void RodGraphicsObject::update()
{
    _updatePolyData();

}

void RodGraphicsObject::_generateInitialPolyData()
{
    const std::vector<Rod::XPBDRod::Node>& nodes = _rod->nodes();
    const std::vector<Vec3r>& cross_section_points = _rod->crossSection()->crossSectionPoints();

    // insert cross-section vertices
    vtkNew<vtkPoints> points;
    for (unsigned ni = 0; ni < nodes.size(); ni++)
    {
        // for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
        for (const auto& p : cross_section_points)
        {
            // transform each point in local cross section to global frame
            const Vec3r transformed = nodes[ni].orientation * p + nodes[ni].position;
            points->InsertNextPoint( transformed[0], transformed[1], transformed[2] );
        }
    }

    // create faces inbetween nodes
    vtkNew<vtkCellArray> faces;
    for (unsigned ni = 0; ni < nodes.size()-1; ni++)
    {
        for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
        {
            // vertices for the quad face
            const int v1 = ni*cross_section_points.size() + pi;
            const int v2 = (pi != cross_section_points.size()-1) ? v1 + 1 : ni*cross_section_points.size();
            const int v3 = v2 + cross_section_points.size();
            const int v4 = v1 + cross_section_points.size();

            // create VTK quad
            vtkNew<vtkQuad> quad;
            quad->GetPointIds()->SetId(0, v1);
            quad->GetPointIds()->SetId(1, v2);
            quad->GetPointIds()->SetId(2, v3);
            quad->GetPointIds()->SetId(3, v4);
            faces->InsertNextCell(quad);
        }
    }

    // end cap faces
    vtkNew<vtkPolygon> base_face, end_face;
    base_face->GetPointIds()->SetNumberOfIds(cross_section_points.size());
    end_face->GetPointIds()->SetNumberOfIds(cross_section_points.size());
    for (unsigned i = 0; i < cross_section_points.size(); i++)
    {
        base_face->GetPointIds()->SetId(i, i);
        end_face->GetPointIds()->SetId(i, (nodes.size()-1)*cross_section_points.size() + i);
    }
    faces->InsertNextCell(base_face);
    faces->InsertNextCell(end_face);

    // vtkNew<vtkPolyData> rod_poly_data;
    _vtk_poly_data->SetPoints(points);
    _vtk_poly_data->SetPolys(faces);
}

void RodGraphicsObject::_updatePolyData()
{
    // TODO: use vtkPoints::setData and vtkFloatArray? supposed to be faster

    const std::vector<Rod::XPBDRod::Node>& nodes = _rod->nodes();
    const std::vector<Vec3r>& cross_section_points = _rod->crossSection()->crossSectionPoints();

    vtkPoints* points = _vtk_poly_data->GetPoints();
    for (unsigned ni = 0; ni < nodes.size(); ni++)
    {
        // for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
        for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
        {
            // transform each point in local cross section to global frame
            const Vec3r transformed = nodes[ni].orientation * cross_section_points[pi] + nodes[ni].position;
            points->SetPoint( ni*cross_section_points.size() + pi, transformed.data() );
        }
    }
    points->Modified();
}


} // namespace Graphics