#include "graphics/GraphicsScene.hpp"

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

namespace Graphics
{

GraphicsScene::GraphicsScene()
    : _should_render(false)
{

}

void GraphicsScene::renderCallback(vtkObject* /*caller*/, long unsigned int /*event_id*/, void* client_data, void* /*call_data*/)
{
    
    GraphicsScene* scene = static_cast<GraphicsScene*>(client_data);
    // std::cout << "Callback! t=" << simulation->_time << std::endl;
    if (scene->_should_render.exchange(false))
    {
        // std::cout << "Rendering... t=" << simulation->_time << std::endl;
        scene->_render_window->Render();
    }
}


void GraphicsScene::setup()
{
    // create renderer for actors in the scene
    _renderer = vtkSmartPointer<vtkOpenGLRenderer>::New();
    
    for (const auto& graphics_obj : _rod_graphics_objects)
    {
        _renderer->AddActor(graphics_obj.getVtkActor());
    }
    
    _renderer->SetBackground(1.0, 1.0, 1.0);

    _renderer->SetAutomaticLightCreation(false);

    vtkNew<vtkTexture> hdr_texture;
    vtkNew<vtkHDRReader> reader;
    reader->SetFileName("../resource/studio_1k.hdr");
    hdr_texture->SetInputConnection(reader->GetOutputPort());
    hdr_texture->SetColorModeToDirectScalars();
    hdr_texture->MipmapOn();
    hdr_texture->InterpolateOn();

    vtkNew<vtkSkybox> skybox;
    skybox->SetTexture(hdr_texture);
    skybox->SetFloorRight(0,0,1);
    skybox->SetProjection(vtkSkybox::Sphere);
    _renderer->AddActor(skybox);

    _renderer->UseImageBasedLightingOn();
    _renderer->UseSphericalHarmonicsOn();
    _renderer->SetEnvironmentTexture(hdr_texture, false);

    vtkNew<vtkLight> light;
    light->SetPosition(0.0, 10, 0.0);
    light->SetFocalPoint(0, 0, 0);
    light->SetColor(1.0, 1.0, 1.0);
    light->SetIntensity(1.0);
    _renderer->AddLight(light);

    vtkNew<vtkPlaneSource> plane;
    plane->SetCenter(0.0, 0.0, 0.0);
    plane->SetNormal(0.0, 1.0, 0.0);
    // plane->SetResolution(10, 10);
    plane->Update();

    vtkNew<vtkPolyDataMapper> plane_mapper;
    plane_mapper->SetInputData(plane->GetOutput());

    vtkNew<vtkPNGReader> plane_tex_reader;
    plane_tex_reader->SetFileName("../resource/textures/ground_plane_texture.png");
    vtkNew<vtkTexture> plane_color;
    plane_color->UseSRGBColorSpaceOn();
    plane_color->SetMipmap(true);
    plane_color->InterpolateOn();
    plane_color->SetInputConnection(plane_tex_reader->GetOutputPort());

    vtkNew<vtkActor> plane_actor;
    plane_actor->SetMapper(plane_mapper);
    // plane_actor->GetProperty()->SetColor(0.9, 0.9, 0.9);
    plane_actor->GetProperty()->SetInterpolationToPBR();
    plane_actor->GetProperty()->SetMetallic(0.0);
    plane_actor->GetProperty()->SetRoughness(0.3);
    plane_actor->SetScale(0.1,1.0,0.1);

    plane_actor->GetProperty()->SetBaseColorTexture(plane_color);
    _renderer->AddActor(plane_actor);

    // create the render window
    _render_window = vtkSmartPointer<vtkRenderWindow>::New();
    _render_window->AddRenderer(_renderer);
    _render_window->SetSize(600, 600);
    _render_window->SetWindowName("Rod Test");

    _interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    _interactor->SetInteractorStyle(style);
    _interactor->SetRenderWindow(_render_window);

    // rendering settings
    _render_window->SetMultiSamples(10);
    
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

    _renderer->SetPass(toneMappingP);

    vtkNew<vtkCallbackCommand> render_callback;
    render_callback->SetCallback(GraphicsScene::renderCallback);
    render_callback->SetClientData(this);
    _interactor->Initialize();
    _interactor->AddObserver(vtkCommand::TimerEvent, render_callback);
    _interactor->CreateRepeatingTimer(5);
}

void GraphicsScene::update()
{
    for (auto& obj : _rod_graphics_objects)
    {
        obj.update();
    }

    _should_render.store(true);
}

void GraphicsScene::addObject(const Rod::XPBDRod* rod)
{
    _rod_graphics_objects.emplace_back(rod);
    _rod_graphics_objects.back().setup();

    _renderer->AddActor(_rod_graphics_objects.back().getVtkActor());
}

} // namespace Graphics