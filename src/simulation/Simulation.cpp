#include "simulation/Simulation.hpp"

#include <chrono>
#include <thread>

#include <vtkCallbackCommand.h>

namespace Sim
{

Simulation::Simulation()
    : _setup(false),
      _time(0), _time_step(1e-3), _end_time(100),
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
    Rod::CircleCrossSection cross_section(0.1, 20);
    Rod::XPBDRod rod(100, 10.0, 1150, 0.05e9, 0.02e9, Vec3r(0,0,0), Mat3r::Identity(), cross_section);

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

    // _renderer->SetAutomaticLightCreation(false);


    // create environment texture
    // vtkNew<vtkImageData> env_image;
    // env_image->SetDimensions(256, 256, 1);
    // env_image->AllocateScalars(VTK_FLOAT, 3);

    // // Fill with uniform white HDR values
    // vtkSmartPointer<vtkFloatArray> scalars = vtkFloatArray::SafeDownCast(env_image->GetPointData()->GetScalars());
    // for (int i = 0; i < 256 * 256; i++) 
    // {
    //     scalars->SetTuple3(i, 0.3f, 0.3f, 0.3f);  // Uniform white (you can increase for brighter)
    // }

    // vtkNew<vtkTexture> env_texture;
    // env_texture->SetInputData(env_image);
    // env_texture->SetColorModeToDirectScalars();
    // env_texture->MipmapOn();
    // env_texture->InterpolateOn();


    // Enable PBR rendering features
    // renderer->UseImageBasedLightingOn();     // Enable IBL
    // renderer->UseSphericalHarmonicsOff();     // Enable spherical harmonics for diffuse IBL
    // renderer->SetEnvironmentTexture(env_texture, true);

    // vtkNew<vtkLight> light;
    // light->SetPosition(1.0, 0, 0.5);
    // light->SetFocalPoint(0, 0, 0.5);
    // light->SetColor(1.0, 1.0, 1.0);
    // light->SetIntensity(3.0);
    // light->SetLightType(VTK_LIGHT_TYPE_SCENE_LIGHT);
    // renderer->AddLight(light);

    // create the render window
    _render_window = vtkSmartPointer<vtkRenderWindow>::New();
    _render_window->AddRenderer(_renderer);
    _render_window->SetSize(600, 600);
    _render_window->SetWindowName("Rod Test");

    _interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    _interactor->SetInteractorStyle(style);
    _interactor->SetRenderWindow(_render_window);

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
    std::cout << "\n\n\nt=" << _time << std::endl;
    for (auto& rod : _rods)
    {
        rod.update(_time_step, _g_accel);
        
        const Vec3r& tip_pos = rod.nodes().back().position;
        const Mat3r& tip_or = rod.nodes().back().orientation;
        std::cout << "Tip position: " << tip_pos[0] << ", " << tip_pos[1] << ", " << tip_pos[2] << std::endl;
        std::cout << "Tip orientation:\n" << tip_or << std::endl;
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