#ifndef __GRAPHICS_SCENE_HPP
#define __GRAPHICS_SCENE_HPP

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

namespace Graphics
{

class GraphicsScene
{
    public:
    static void renderCallback(vtkObject* caller, long unsigned int event_id, void* client_data, void* call_data);
    void displayWindow() { _render_window->Render(); }
    void interactorStart() { _interactor->Start(); }

    explicit GraphicsScene();

    void setup();

    void update();

    void addObject(const Rod::XPBDRod* rod);

    private:
    vtkSmartPointer<vtkOpenGLRenderer> _renderer;
    vtkSmartPointer<vtkRenderWindow> _render_window;
    vtkSmartPointer<vtkRenderWindowInteractor> _interactor;

    std::vector<Graphics::RodGraphicsObject> _rod_graphics_objects;

    std::atomic<bool> _should_render;
};

} // namespace Graphics

#endif // __GRAPHICS_SCENE_HPP