/*=========================================================================

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "rod/XPBDRod.hpp"

#include <math.h>

// First include the required header files for the VTK classes we are using.
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkPolyDataNormals.h>
#include <vtkLight.h>
#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkTexture.h>
#include <vtkPointData.h>

void createRodPolyData(const Rod::XPBDRod* rod, vtkPolyData* rod_poly_data)
{
    const std::vector<Rod::XPBDRod::Node>& nodes = rod->nodes();
    const std::vector<Vec3r>& cross_section_points = rod->crossSection()->crossSectionPoints();
    int num_vertices = nodes.size() * cross_section_points.size() + 2;

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

    // insert 2 vertices to make up the end faces
    points->InsertNextPoint( nodes[0].position[0], nodes[0].position[1], nodes[0].position[2] );
    points->InsertNextPoint( nodes.back().position[0], nodes.back().position[1], nodes.back().position[2] );

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
    rod_poly_data->SetPoints(points);
    rod_poly_data->SetPolys(faces);
    
    // return rod_poly_data.Get();
}

int main(int, char*[])
{
    //
    // Next we create an instance of vtkNamedColors and we will use
    // this to select colors for the object and background.
    //
    vtkNew<vtkNamedColors> colors;

    Rod::CircleCrossSection cross_section(0.1, 30);
    Rod::XPBDRod rod(20, 1.0, Vec3r(0,0,0), Mat3r::Identity(), &cross_section);

    // get poly data for rod
    vtkNew<vtkPolyData> rod_poly_data;
    createRodPolyData(&rod, rod_poly_data.Get());

    vtkNew<vtkPolyDataNormals> rod_normals;
    rod_normals->SetInputData(rod_poly_data);
    rod_normals->ComputePointNormalsOn();
    rod_normals->ComputeCellNormalsOn();
    rod_normals->SplittingOn();
    // rod_normals->FlipNormalsOn();
    rod_normals->Update();

    // poly data mapper for rod
    vtkNew<vtkPolyDataMapper> rod_mapper;
    // rod_mapper->SetInputData(rod_poly_data);
    rod_mapper->SetInputConnection(rod_normals->GetOutputPort());

    // create the actor for the rod
    vtkNew<vtkActor> rod_actor;
    rod_actor->SetMapper(rod_mapper);
    rod_actor->GetProperty()->SetInterpolationToPBR();
    // rod_actor->GetProperty()->SetInterpolationToPhong();
    rod_actor->GetProperty()->SetColor(0.75, 0.75, 0.75); // Base color (albedo)
    
    // PBR Material Properties - adjusted for better visibility
    rod_actor->GetProperty()->SetMetallic(1.0); 
    rod_actor->GetProperty()->SetRoughness(0.3);       
    rod_actor->GetProperty()->SetAnisotropy(0.0);     
    rod_actor->GetProperty()->SetBaseIOR(1.5);  
    
    rod_actor->GetProperty()->SetOcclusionStrength(1.0);
    
    // create renderer for actors in the scene
    vtkNew<vtkOpenGLRenderer> renderer;
    // vtkNew<vtkRenderer> renderer;
    renderer->AddActor(rod_actor);
    renderer->SetBackground(1.0, 1.0, 1.0);

    renderer->SetAutomaticLightCreation(false);


    // create environment texture
    vtkNew<vtkImageData> env_image;
    env_image->SetDimensions(256, 256, 1);
    env_image->AllocateScalars(VTK_FLOAT, 3);

    // Fill with uniform white HDR values
    vtkSmartPointer<vtkFloatArray> scalars = vtkFloatArray::SafeDownCast(env_image->GetPointData()->GetScalars());
    for (int i = 0; i < 256 * 256; i++) 
    {
        scalars->SetTuple3(i, 0.3f, 0.3f, 0.3f);  // Uniform white (you can increase for brighter)
    }

    vtkNew<vtkTexture> env_texture;
    env_texture->SetInputData(env_image);
    env_texture->SetColorModeToDirectScalars();
    env_texture->MipmapOn();
    env_texture->InterpolateOn();


    // Enable PBR rendering features
    renderer->UseImageBasedLightingOn();     // Enable IBL
    renderer->UseSphericalHarmonicsOff();     // Enable spherical harmonics for diffuse IBL
    renderer->SetEnvironmentTexture(env_texture, true);

    vtkNew<vtkLight> light;
    light->SetPosition(1.0, 0, 0.5);
    light->SetFocalPoint(0, 0, 0.5);
    light->SetColor(1.0, 1.0, 1.0);
    light->SetIntensity(3.0);
    light->SetLightType(VTK_LIGHT_TYPE_SCENE_LIGHT);
    renderer->AddLight(light);

    // vtkNew<vtkLight> light2;
    // light->SetPosition(-1.0, 0, 0.5);
    // light->SetFocalPoint(0, 0, 0.5);
    // light->SetColor(1.0, 1.0, 1.0);
    // light->SetIntensity(5.0);
    // light->SetLightType(VTK_LIGHT_TYPE_SCENE_LIGHT);
    // renderer->AddLight(light2);

    // vtkNew<vtkLight> light3;
    // light->SetPosition(0, 1.0, 0.5);
    // light->SetFocalPoint(0, 0, 0.5);
    // light->SetColor(1.0, 1.0, 1.0);
    // light->SetIntensity(5.0);
    // light->SetLightType(VTK_LIGHT_TYPE_SCENE_LIGHT);
    // renderer->AddLight(light3);

    // vtkNew<vtkLight> light4;
    // light->SetPosition(0, -1.0, 0.5);
    // light->SetFocalPoint(0, 0, 0.5);
    // light->SetColor(1.0, 1.0, 1.0);
    // light->SetIntensity(5.0);
    // light->SetLightType(VTK_LIGHT_TYPE_SCENE_LIGHT);
    // renderer->AddLight(light4);

    // create the render window
    vtkNew<vtkRenderWindow> render_window;
    render_window->AddRenderer(renderer);
    render_window->SetSize(600, 600);
    render_window->SetWindowName("Rod Test");

    vtkNew<vtkRenderWindowInteractor> interactor;
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor->SetInteractorStyle(style);
    interactor->SetRenderWindow(render_window);
    render_window->Render();

    interactor->Start();

    return EXIT_SUCCESS;
}
