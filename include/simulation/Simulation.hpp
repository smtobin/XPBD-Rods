#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include "common/common.hpp"

#include "rod/XPBDRod.hpp"
#include "graphics/RodGraphicsObject.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkCallbackCommand.h>

#include <vector>
#include <atomic>

namespace Sim
{

class Simulation
{
    public:
    explicit Simulation();

    static void renderCallback(vtkObject* caller, long unsigned int event_id, void* client_data, void* call_data);

    void setup();

    int run();

    void update();

    void interactorStart() { _interactor->Start(); }

    private:
    void _createRodPolyData(const Rod::XPBDRod* rod, vtkPolyData* rod_poly_data);

    void _timeStep();

    void _updateGraphics();

    private:
    bool _setup;

    Real _time;
    Real _time_step;
    Real _end_time;
    Real _g_accel;
    int _viewer_refresh_time_ms;

    std::vector<Rod::XPBDRod> _rods;

    // graphics
    vtkSmartPointer<vtkOpenGLRenderer> _renderer;
    vtkSmartPointer<vtkRenderWindow> _render_window;
    vtkSmartPointer<vtkRenderWindowInteractor> _interactor;

    std::vector<Graphics::RodGraphicsObject> _rod_graphics_objects;

    std::atomic<bool> _should_render;
};

} // namespace Simulation

#endif // __SIMULATION_HPP