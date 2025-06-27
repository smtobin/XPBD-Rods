#include "simulation/Simulation.hpp"

#include <chrono>
#include <thread>

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

namespace Sim
{

Simulation::Simulation()
    : _setup(false),
      _time(0), _time_step(1e-3), _end_time(10),
      _g_accel(9.81), _viewer_refresh_time_ms(1000.0/30.0),
      _should_render(false)
{

}

void Simulation::renderCallback(vtkObject* /*caller*/, long unsigned int /*event_id*/, void* client_data, void* /*call_data*/)
{
    
    Simulation* simulation = static_cast<Simulation*>(client_data);
    // std::cout << "Callback! t=" << simulation->_time << std::endl;
    if (simulation->_should_render.exchange(false))
    {
        // std::cout << "Rendering... t=" << simulation->_time << std::endl;
        simulation->_render_window->Render();
    }
    
       
}


void Simulation::setup()
{
    _setup = true;

    // create rod(s)
    Rod::CircleCrossSection cross_section(0.00156, 20);
    Rod::XPBDRod rod(1000, 0.1, 650, 60e9, Vec3r(0,0.05,0), Mat3r::Identity(), cross_section);

    _rods.push_back(std::move(rod));
    _rods.back().setup();
    _rod_graphics_objects.emplace_back(&_rods.back());
    _rod_graphics_objects.back().setup();

    // setup graphics

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

    // _render_window->Render();
    // _interactor->Start();
}

void Simulation::update()
{
    auto wall_time_start = std::chrono::steady_clock::now();
    auto last_redraw = std::chrono::steady_clock::now();

    while (_time < _end_time)
    {
        // the elapsed seconds in wall time since the simulation has started
        Real wall_time_elapsed_s = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - wall_time_start).count() / 1000000000.0;
        
        // if the simulation is ahead of the current elapsed wall time, stall
        if (_time > wall_time_elapsed_s)
        {
            continue;
        }

        _timeStep();

        // the time in ms since the viewer was last redrawn
        auto time_since_last_redraw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_redraw).count();
        // we want ~30 fps, so update the viewer every 33 ms
        if (time_since_last_redraw_ms > _viewer_refresh_time_ms)
        {
            // std::cout << _haptic_device_manager->getPosition() << std::endl;
            _updateGraphics();

            last_redraw = std::chrono::steady_clock::now();
        }
    }

    auto wall_time_end = std::chrono::steady_clock::now();
    std::cout << "Simulation " << _end_time << " seconds took " << std::chrono::duration_cast<std::chrono::milliseconds>(wall_time_end - wall_time_start).count() << " ms" << std::endl;
}

int Simulation::run()
{
    // setup if we haven't already
    if (!_setup)
        setup();

    vtkNew<vtkCallbackCommand> render_callback;
    render_callback->SetCallback(Simulation::renderCallback);
    render_callback->SetClientData(this);
    _interactor->Initialize();
    _interactor->AddObserver(vtkCommand::TimerEvent, render_callback);
    _interactor->CreateRepeatingTimer(5);
    

    // start graphics window
    _render_window->Render();
    std::thread update_thread(&Simulation::update, this);
    
    _interactor->Start();
    // update_thread.join();
}

void Simulation::_timeStep()
{
    std::cout << "t=" << _time << std::endl;
    for (auto& rod : _rods)
    {
        rod.update(_time_step, _g_accel);
        
        const Vec3r& tip_pos = rod.nodes().back().position;
        const Mat3r& tip_or = rod.nodes().back().orientation;
        // std::cout << "Tip position: " << tip_pos[0] << ", " << tip_pos[1] << ", " << tip_pos[2] << std::endl;
        // std::cout << "Tip orientation:\n" << tip_or << std::endl;
    }

    _time += _time_step;
}

void Simulation::_updateGraphics()
{
    for (auto& graphics_obj : _rod_graphics_objects)
    {
        graphics_obj.update();
    }

    _should_render.store(true);
    // _interactor->Modified(); // will invoke ModifiedEvent which is set up to render
}

} // namespace Simulation