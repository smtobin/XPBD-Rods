#include "graphics/RodGraphicsObject.hpp"

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

    // vtkNew<vtkTriangleFilter> triangulation;
    // triangulation->SetInputData(_vtk_poly_data);

    // vtkNew<vtkCleanPolyData> cleaner;
    // cleaner->SetInputConnection(triangulation->GetOutputPort());
    // cleaner->Update();

    // Add this step for smooth normals
    vtkNew<vtkPolyDataNormals> normalGenerator;
    // normalGenerator->SetInputConnection(cleaner->GetOutputPort());
    normalGenerator->SetInputData(_vtk_poly_data);
    normalGenerator->SetFeatureAngle(30.0); // Force smooth normals
    normalGenerator->SplittingOff();
    normalGenerator->ConsistencyOn();
    normalGenerator->AutoOrientNormalsOn();
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOff();
    normalGenerator->FlipNormalsOff();
    normalGenerator->Update();

    vtkNew<vtkPolyDataTangents> tangents;
    tangents->SetInputConnection(normalGenerator->GetOutputPort());
    // tangents->SetInputData(_vtk_poly_data);
    tangents->SetComputePointTangents(true);
    tangents->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(tangents->GetOutputPort());

    std::filesystem::path pbr_texture_folder("../resource/textures/Metal050A_4K-PNG");
    std::string orm_filename = pbr_texture_folder.filename().string() + "_ORM.png";
    std::string normals_filename = pbr_texture_folder.filename().string() + "_NormalGL.png";
    std::string color_filename = pbr_texture_folder.filename().string() + "_Color.png";
    
    std::filesystem::path orm_file(pbr_texture_folder); orm_file.append(orm_filename);
    std::filesystem::path normals_file(pbr_texture_folder); normals_file.append(normals_filename);
    std::filesystem::path color_file(pbr_texture_folder); color_file.append(color_filename);

    vtkNew<vtkPNGReader> orm_reader;
    orm_reader->SetFileName(orm_file.c_str());
    vtkNew<vtkTexture> material;
    material->InterpolateOn();
    material->SetInputConnection(orm_reader->GetOutputPort());

    vtkNew<vtkPNGReader> color_reader;
    color_reader->SetFileName(color_file.c_str());
    vtkNew<vtkTexture> color;
    color->UseSRGBColorSpaceOn();
    color->InterpolateOn();
    color->SetInputConnection(color_reader->GetOutputPort());

    vtkNew<vtkPNGReader> normals_reader;
    normals_reader->SetFileName(normals_file.c_str());
    vtkNew<vtkTexture> normals;
    normals->SetMipmap(true);
    normals->InterpolateOn();
    normals->SetMaximumAnisotropicFiltering(8);
    normals->SetInputConnection(normals_reader->GetOutputPort());

    _vtk_actor->SetMapper(mapper);
    _vtk_actor->GetProperty()->SetInterpolationToPBR();
    // _vtk_actor->GetProperty()->SetInterpolationToPhong();
    // actor->GetProperty()->SetColor(0.75, 0.75, 0.75);
    // actor->GetProperty()->SetMetallic(1.0);
    // actor->GetProperty()->SetRoughness(0.0);
    _vtk_actor->GetProperty()->SetMetallic(1.0);
    _vtk_actor->GetProperty()->SetRoughness(1.0);
    // actor->GetProperty()->SetNormalScale(0.1);
    // actor->GetProperty()->SetSpecular(0.0);
    // actor->GetProperty()->SetSpecularPower(1.0);
    // actor->GetProperty()->SetCoatColor(colors->GetColor3d("White").GetData());

    _vtk_actor->GetProperty()->SetBaseColorTexture(color);
    _vtk_actor->GetProperty()->SetORMTexture(material);
    // _vtk_actor->GetProperty()->SetNormalTexture(normals);
    // actor->GetProperty()->SetNormalTexture(nullptr);

    // Create a simple test normal texture
    // vtkNew<vtkImageData> testNormalMap;
    // testNormalMap->SetDimensions(256, 256, 1);
    // testNormalMap->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

    // unsigned char* pixels = static_cast<unsigned char*>(testNormalMap->GetScalarPointer());
    // for (int i = 0; i < 256 * 256; i++) {
    //     pixels[i*3 + 0] = 128; // X normal
    //     pixels[i*3 + 1] = 128; // Y normal  
    //     pixels[i*3 + 2] = 255; // Z normal (pointing out)
    // }

    // vtkNew<vtkTexture> normalTexture;
    // normalTexture->SetInputData(testNormalMap);
    // _vtk_actor->GetProperty()->SetNormalTexture(normalTexture);
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
            // vtkNew<vtkQuad> quad;
            // quad->GetPointIds()->SetId(0, v1);
            // quad->GetPointIds()->SetId(1, v2);
            // quad->GetPointIds()->SetId(2, v3);
            // quad->GetPointIds()->SetId(3, v4);

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
    }

    // end cap points
    points->InsertNextPoint( nodes[0].position[0], nodes[0].position[1], nodes[0].position[2] );
    points->InsertNextPoint( nodes.back().position[0], nodes.back().position[1], nodes.back().position[2] );

    // end cap faces
    // base faces
    for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
    {
        const int v1 = pi;
        const int v2 = nodes.size()*cross_section_points.size();
        const int v3 = (pi != cross_section_points.size()-1) ? v1 + 1 : 0;

        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, v1);
        tri->GetPointIds()->SetId(1, v2);
        tri->GetPointIds()->SetId(2, v3);
        faces->InsertNextCell(tri);
    }

    // tip faces
    for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
    {
        const int v1 = (nodes.size()-1)*cross_section_points.size() + pi;
        const int v2 = (pi != cross_section_points.size()-1) ? v1 + 1 : (nodes.size()-1)*cross_section_points.size();
        const int v3 = nodes.size()*cross_section_points.size() + 1;
        

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
    vtkNew<vtkFloatArray> textureCoords;
    textureCoords->SetNumberOfComponents(2);
    textureCoords->SetNumberOfTuples(nodes.size()*cross_section_points.size());
    textureCoords->SetName("TextureCoordinates");
    for (unsigned ni = 0; ni < nodes.size(); ni++)
    {
        float x_coord = (ni % 2 == 0) ? 0.0f : 0.5f;
        for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
        {
            float y_coord = static_cast<float>(pi) / cross_section_points.size();
            textureCoords->SetTuple2(ni*cross_section_points.size() + pi, x_coord, y_coord);
        }
    }
    _vtk_poly_data->GetPointData()->SetTCoords(textureCoords);
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
    points->SetPoint(nodes.size()*cross_section_points.size(), nodes[0].position[0], nodes[0].position[1], nodes[0].position[2]);
    points->SetPoint(nodes.size()*cross_section_points.size()+1, nodes.back().position[0], nodes.back().position[1], nodes.back().position[2]);
    points->Modified();
}


} // namespace Graphics