#pragma once

#include "simobject/rod/XPBDRod.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "graphics/GraphicsObject.hpp"
#include "graphics/SphereGraphicsObject.hpp"
#include "graphics/BoxGraphicsObject.hpp"
#include "graphics/RodGraphicsObject.hpp"
#include "graphics/HigherOrderRodGraphicsObject.hpp"
#include "graphics/PlaneGraphicsObject.hpp"

#include "config/SimulationRenderConfig.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkCallbackCommand.h>

#include <vector>
#include <atomic>
#include <memory>

namespace Sim
{
    class Simulation;
}

namespace Graphics
{

class GraphicsScene
{
    public:
    static void renderCallback(vtkObject* caller, long unsigned int event_id, void* client_data, void* call_data);
    void displayWindow() { _render_window->Render(); }
    void interactorStart() { _interactor->Start(); }

    explicit GraphicsScene();
    explicit GraphicsScene(const Config::SimulationRenderConfig& sim_render_config);

    void setup(Sim::Simulation* sim=nullptr);

    void update();

    template <int Order>
    void addObject(const SimObject::XPBDRod_<Order>* rod, const Config::ObjectRenderConfig& render_config)
    {
        std::unique_ptr<HigherOrderRodGraphicsObject<Order>> rod_go = std::make_unique<HigherOrderRodGraphicsObject<Order>>(rod, render_config);

        _renderer->AddActor(rod_go->actor());
        _graphics_objects.push_back(std::move(rod_go));
    }
    void addObject(const SimObject::XPBDRod* rod, const Config::ObjectRenderConfig& render_config);
    void addObject(const SimObject::XPBDRigidSphere* sphere, const Config::ObjectRenderConfig& render_config);
    void addObject(const SimObject::XPBDRigidBox* box, const Config::ObjectRenderConfig& render_config);
    void addObject(const SimObject::XPBDPlane* plane, const Config::ObjectRenderConfig& render_config);
    void addObject(const SimObject::XPBDObjectGroup_Base* pen, const Config::ObjectRenderConfig& render_config);

    Vec3r cameraPosition() const;
    Vec3r cameraViewDirection() const;
    Vec3r cameraUpDirection() const;
    Vec2r worldCoordinatesToPixelCoordinates(const Vec3r& world) const;

    private:
    vtkSmartPointer<vtkOpenGLRenderer> _renderer;
    vtkSmartPointer<vtkRenderWindow> _render_window;
    vtkSmartPointer<vtkRenderWindowInteractor> _interactor;

    std::vector<std::unique_ptr<GraphicsObject>> _graphics_objects;

    std::atomic<bool> _should_render;

    Config::SimulationRenderConfig _render_config;

};

} // namespace Graphics