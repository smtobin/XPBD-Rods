#ifndef __VTK_CUSTOM_INTERACTOR_STYLE_HPP
#define __VTK_CUSTOM_INTERACTOR_STYLE_HPP

#include <vtkInteractorStyleTrackballCamera.h>
#include <functional>

class vtkCustomInteractorStyle : public vtkInteractorStyleTrackballCamera
{
    public:
    static vtkCustomInteractorStyle* New();
    vtkTypeMacro(vtkCustomInteractorStyle, vtkInteractorStyleTrackballCamera);

    void SetUpdateCondition(std::function<bool()> condition)
    {
        this->update_condition = condition;
    }

    void SetRenderCallback(std::function<void()> callback)
    {
        this->render_callback = callback;
    }

    protected:
    vtkCustomInteractorStyle() {}
    
    private:
    std::function<bool()> update_condition;
    std::function<void()> render_callback;
};

#endif // __VTK_CUSTOM_INTERACTOR_STYLE_HPP