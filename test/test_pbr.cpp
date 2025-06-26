#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>
#include <vtkHDRReader.h>
#include <vtkImageFlip.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLight.h>
#include <vtkNamedColors.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLTexture.h>
#include <vtkPBRIrradianceTexture.h>
#include <vtkPNGReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataTangents.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSkybox.h>
#include <vtkTexture.h>
#include <vtkTriangleFilter.h>

#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkShadowMapPass.h>
#include <vtkCameraPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderStepsPass.h>
#include <vtkToneMappingPass.h>
#include <vtkLightsPass.h>
#include <vtkOpaquePass.h>

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

int main()
{
    vtkNew<vtkNamedColors> colors;
    colors->SetColor("Brass", std::array<unsigned char, 4>{184, 115, 51, 255}.data());

    vtkNew<vtkOpenGLRenderer> renderer;
    renderer->SetBackground(colors->GetColor3d("Black").GetData());
    renderer->AutomaticLightCreationOff();

    vtkNew<vtkLight> light;
    light->SetPosition(0.0, 5.0, 0.0);
    light->SetFocalPoint(0.0, 0.0, 0.0);
    light->SetIntensity(10);
    renderer->AddLight(light);

    vtkNew<vtkRenderWindow> render_window;
    render_window->SetSize(600, 600);
    render_window->AddRenderer(renderer);

    vtkNew<vtkRenderWindowInteractor> interactor;
    interactor->SetRenderWindow(render_window);

    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor->SetInteractorStyle(style);

    vtkSmartPointer<vtkPBRIrradianceTexture> irradiance = renderer->GetEnvMapIrradiance();
    irradiance->SetIrradianceStep(0.3);
    renderer->UseSphericalHarmonicsOff();

    

    ///////////////////////////////////////////////////
    // SET UP HDR AND SKYBOX
    //////////////////////////////////////////////////
    vtkNew<vtkTexture> hdr_texture;
    vtkNew<vtkHDRReader> reader;
    reader->SetFileName("../resource/sky.hdr");
    hdr_texture->SetInputConnection(reader->GetOutputPort());
    hdr_texture->SetColorModeToDirectScalars();
    hdr_texture->MipmapOn();
    hdr_texture->InterpolateOn();

    vtkNew<vtkSkybox> skybox;
    skybox->SetTexture(hdr_texture);
    skybox->SetFloorRight(0,0,1);
    skybox->SetProjection(vtkSkybox::Sphere);
    renderer->AddActor(skybox);

    renderer->UseImageBasedLightingOn();
    renderer->UseSphericalHarmonicsOn();
    renderer->SetEnvironmentTexture(hdr_texture, false);


    ///////////////////////////////////////////////////
    // SET UP SPHERE
    ///////////////////////////////////////////////////
    // vtkNew<vtkSphereSource> sphere;
    // sphere->SetThetaResolution(75);
    // sphere->SetPhiResolution(75);
    vtkNew<vtkCubeSource> cube;


    vtkNew<vtkTriangleFilter> triangulation;
    triangulation->SetInputConnection(cube->GetOutputPort());

    vtkNew<vtkPolyDataTangents> tangents;
    tangents->SetInputConnection(triangulation->GetOutputPort());

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(tangents->GetOutputPort());


    /////////////////////////////////////////////
    // READ PBR TEXTURE
    /////////////////////////////////////////////
    std::filesystem::path pbr_texture_folder("../resource/textures/Flesh01_4k");
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
    // normals->SetMinificationFilter(vtkTexture::Linear);
    // normals->SetMagnificationFilter(vtkTexture::Linear);
    

    vtkNew<vtkActor> actor;
    actor->SetPosition(0.0, 0.8, 0.0);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetInterpolationToPBR();
    // actor->GetProperty()->SetColor(0.75, 0.75, 0.75);
    // actor->GetProperty()->SetMetallic(1.0);
    // actor->GetProperty()->SetRoughness(0.0);
    actor->GetProperty()->SetMetallic(0.0);
    actor->GetProperty()->SetRoughness(0.9);
    // actor->GetProperty()->SetNormalScale(0.1);
    // actor->GetProperty()->SetSpecular(0.0);
    // actor->GetProperty()->SetSpecularPower(1.0);
    // actor->GetProperty()->SetCoatColor(colors->GetColor3d("White").GetData());

    actor->GetProperty()->SetBaseColorTexture(color);
    // actor->GetProperty()->SetORMTexture(material);
    actor->GetProperty()->SetNormalTexture(normals);
    // actor->GetProperty()->SetNormalTexture(nullptr);

    renderer->AddActor(actor);

    vtkNew<vtkPlaneSource> plane;
    plane->SetCenter(0.0, 0.0, 0.0);
    plane->SetNormal(0.0, 1.0, 0.0);
    plane->Update();

    vtkNew<vtkPolyDataMapper> plane_mapper;
    plane_mapper->SetInputData(plane->GetOutput());

    vtkNew<vtkActor> plane_actor;
    plane_actor->SetMapper(plane_mapper);
    plane_actor->GetProperty()->SetColor(0.9, 0.9, 0.9);
    plane_actor->GetProperty()->SetInterpolationToPBR();
    plane_actor->GetProperty()->SetMetallic(0.0);
    plane_actor->GetProperty()->SetRoughness(0.3);
    plane_actor->SetScale(2.0,1.0,2.0);
    renderer->AddActor(plane_actor);



    ////////////////////////////////////////////////
    // RENDERING
    ////////////////////////////////////////////////
    render_window->SetMultiSamples(10);

    
    vtkNew<vtkSequencePass> seqP;
    vtkNew<vtkOpaquePass> opaqueP;
    vtkNew<vtkLightsPass> lightsP;

    vtkNew<vtkShadowMapPass> shadows;
    shadows->GetShadowMapBakerPass()->SetResolution(1024);

    vtkNew<vtkRenderPassCollection> passes;
    passes->AddItem(lightsP);
    passes->AddItem(opaqueP);
    passes->AddItem(shadows->GetShadowMapBakerPass());
    passes->AddItem(shadows);
    seqP->SetPasses(passes);

    vtkNew<vtkCameraPass> cameraP;
    cameraP->SetDelegatePass(seqP);

    vtkNew<vtkToneMappingPass> toneMappingP;
    toneMappingP->SetToneMappingType(vtkToneMappingPass::GenericFilmic);
    toneMappingP->SetGenericFilmicDefaultPresets();
    toneMappingP->SetDelegatePass(cameraP);
    toneMappingP->SetExposure(0.5);

    renderer->SetPass(toneMappingP);

    render_window->SetWindowName("PBR Test");
    render_window->Render();
    interactor->Start();

    return EXIT_SUCCESS;

}