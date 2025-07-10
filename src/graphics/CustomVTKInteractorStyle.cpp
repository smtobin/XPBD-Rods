#include "graphics/CustomVTKInteractorStyle.hpp"
#include "simulation/Simulation.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <string>
#include <iostream>

vtkStandardNewMacro(Graphics::CustomVTKInteractorStyle);

namespace Graphics
{

void CustomVTKInteractorStyle::registerSimulation(Sim::Simulation* sim)
{
    _sim = sim;
}

void CustomVTKInteractorStyle::OnKeyPress()
{
    // Get the keypress.
    vtkRenderWindowInteractor* rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    if (_sim)
        _sim->notifyKeyPressed(key);

    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void CustomVTKInteractorStyle::OnKeyRelease()
{
    // Get the key release.
    vtkRenderWindowInteractor* rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    if (_sim)
        _sim->notifyKeyReleased(key);

    vtkInteractorStyleTrackballCamera::OnKeyRelease();
}

void CustomVTKInteractorStyle::OnMouseMove()
{
    vtkRenderWindowInteractor* rwi = this->Interactor;
    int mx, my;
    rwi->GetMousePosition(&mx, &my);

    if (_sim)
        _sim->notifyMouseMoved(mx, my);
    
    vtkInteractorStyleTrackballCamera::OnMouseMove();
}

} // namespace Graphics