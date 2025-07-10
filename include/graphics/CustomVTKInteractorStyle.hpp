#ifndef __CUSTOM_VTK_INTERACTOR_HPP
#define __CUSTOM_VTK_INTERACTOR_HPP

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>

namespace Sim
{
    class Simulation;
}

namespace Graphics
{

class CustomVTKInteractorStyle : public vtkInteractorStyleTrackballCamera
{
    public:
    static CustomVTKInteractorStyle* New();
    vtkTypeMacro(CustomVTKInteractorStyle, vtkInteractorStyleTrackballCamera);

    void registerSimulation(Sim::Simulation* sim);

    virtual void OnKeyPress() override;
    virtual void OnKeyRelease() override;
    virtual void OnMouseMove() override;
    virtual void OnLeftButtonDown() override;
    virtual void OnLeftButtonUp() override;

    private:
    Sim::Simulation* _sim;

};

} // namespace Graphics

#endif // __CUSTOM_VTK_INTERACTOR_HPP