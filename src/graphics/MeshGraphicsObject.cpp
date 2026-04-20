#include "graphics/MeshGraphicsObject.hpp"
#include "graphics/VTKUtils.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>

#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>
#include <vtkCellData.h>

#include <vtkTexture.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataTangents.h>
#include <vtkPNGReader.h>
#include <vtkCleanPolyData.h>
#include <vtkImageData.h>

#include <vtkNew.h>

namespace Graphics
{

MeshGraphicsObject::MeshGraphicsObject(const Mesh* mesh, const SimObject::OrientedParticle* com, const Config::MeshRenderConfig& render_config)
    : GraphicsObject(render_config), _mesh(mesh), _com(com)
{

    vtkNew<vtkPolyData> poly_data;

    // create points
    vtkNew<vtkPoints> vtk_points;
    for (const auto& vertex : _mesh->vertices())
    {
        vtk_points->InsertNextPoint(vertex[0], vertex[1], vertex[2]);
    }

    // create faces
    vtkNew<vtkCellArray> vtk_faces;
    for (const auto& face : _mesh->faces())
    {
        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, face[0]);
        tri->GetPointIds()->SetId(1, face[1]);
        tri->GetPointIds()->SetId(2, face[2]);

        vtk_faces->InsertNextCell(tri);
    }

    poly_data->SetPoints(vtk_points);
    poly_data->SetPolys(vtk_faces);

    if (render_config.drawEdges())
    {
        
        vtkNew<vtkExtractEdges> edge_extractor;
        edge_extractor->SetInputData(poly_data);
        edge_extractor->Update();

        vtkNew<vtkPolyDataMapper> edge_mapper;
        edge_mapper->SetInputConnection(edge_extractor->GetOutputPort());

        _edges_vtk_actor = vtkSmartPointer<vtkActor>::New();
        _edges_vtk_actor->SetMapper(edge_mapper);

        _edges_vtk_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
        _edges_vtk_actor->GetProperty()->LightingOff();
    }

    if (render_config.drawFaces())
    {
        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(poly_data);

        if (render_config.smoothNormals())
        {
            // smooth normals
            vtkNew<vtkPolyDataNormals> normal_generator;
            normal_generator->SetInputData(poly_data);
            normal_generator->SetFeatureAngle(30.0);
            normal_generator->SplittingOff();
            normal_generator->ConsistencyOn();
            normal_generator->ComputePointNormalsOn();
            normal_generator->ComputeCellNormalsOff();
            normal_generator->Update();

            // vtkNew<vtkPolyDataTangents> tangents;
            // tangents->SetInputConnection(normal_generator->GetOutputPort());
            // tangents->Update();

            mapper->SetInputConnection(normal_generator->GetOutputPort());
        }
        else
        {
            mapper->SetInputData(poly_data);
        }

        _vtk_actor = vtkSmartPointer<vtkActor>::New();
        _vtk_actor->SetMapper(mapper);

        VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

        _vtk_transform = vtkSmartPointer<vtkTransform>::New();

        // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
        Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> mesh_transform_mat = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>::Identity();
        mesh_transform_mat.block<3,3>(0,0) = _com->orientation;
        mesh_transform_mat.block<3,1>(0,3) = _com->position;
        _vtk_transform->SetMatrix(mesh_transform_mat.data());

        _vtk_actor->SetUserTransform(_vtk_transform);
        _edges_vtk_actor->SetUserTransform(_vtk_transform);
    }
    
}

void MeshGraphicsObject::update()
{
    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> mesh_transform_mat = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>::Identity();
    mesh_transform_mat.block<3,3>(0,0) = _com->orientation;
    mesh_transform_mat.block<3,1>(0,3) = _com->position;
    _vtk_transform->SetMatrix(mesh_transform_mat.data());
}



} // namespace Graphics