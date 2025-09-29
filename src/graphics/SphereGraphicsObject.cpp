#include "graphics/SphereGraphicsObject.hpp"
#include "graphics/VTKUtils.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkMatrix4x4.h>

namespace Graphics
{

SphereGraphicsObject::SphereGraphicsObject(const SimObject::XPBDRigidSphere* sphere, const Config::ObjectRenderConfig& render_config)
    : GraphicsObject(render_config), _sphere(sphere)
{
     // create the vtkActor from a sphere source
    _sphere_source = vtkSmartPointer<vtkSphereSource>::New();
    _sphere_source->SetRadius(sphere->radius());

    
    vtkNew<vtkPolyDataMapper> data_mapper;

    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(_sphere_source->GetOutputPort());
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        data_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        data_mapper->SetInputConnection(_sphere_source->GetOutputPort());
    }

    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(data_mapper);

    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> sphere_transform_mat = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>::Identity();
    sphere_transform_mat.block<3,3>(0,0) = _sphere->com().orientation;
    sphere_transform_mat.block<3,1>(0,3) = _sphere->com().position;
    _vtk_transform->SetMatrix(sphere_transform_mat.data());

    _vtk_actor->SetUserTransform(_vtk_transform);
}

void SphereGraphicsObject::update()
{
    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> sphere_transform_mat = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>::Identity();
    sphere_transform_mat.block<3,3>(0,0) = _sphere->com().orientation;
    sphere_transform_mat.block<3,1>(0,3) = _sphere->com().position;
    _vtk_transform->SetMatrix(sphere_transform_mat.data());
}



} // namespace Graphics