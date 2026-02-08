#include "graphics/PlaneGraphicsObject.hpp"
#include "graphics/VTKUtils.hpp"

#include "simobject/rigidbody/XPBDPlane.hpp"

#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkImageData.h>
#include <vtkTexture.h>
#include <vtkProperty.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>

namespace Graphics
{

PlaneGraphicsObject::PlaneGraphicsObject(const SimObject::XPBDPlane* plane, const Config::ObjectRenderConfig& render_config)
    : GraphicsObject(render_config), _plane(plane)
{
    // Create the plane geometry
    _plane_source = vtkSmartPointer<vtkPlaneSource>::New();
    _plane_source->SetOrigin(-_plane->width()/2, -_plane->height()/2, 0);
    _plane_source->SetPoint1(_plane->width()/2, -_plane->height()/2, 0);
    _plane_source->SetPoint2(-_plane->width()/2, _plane->height()/2, 0);
    _plane_source->Update();
    
    // Create procedural checker texture
    int tex_size = 2;  // 2x2 texture that will tile
    vtkNew<vtkImageData> image_data;
    image_data->SetDimensions(tex_size, tex_size, 1);
    image_data->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    
    // Fill with checker pattern (white and gray)
    for (int i = 0; i < tex_size; i++) {
        for (int j = 0; j < tex_size; j++) {
            unsigned char* pixel = static_cast<unsigned char*>(
                image_data->GetScalarPointer(i, j, 0));
            
            bool is_white = (i + j) % 2 == 0;
            unsigned char color = is_white ? 255 : 180;  // White or light gray
            pixel[0] = color;
            pixel[1] = color;
            pixel[2] = color;
        }
    }
    
    // Create texture and set repeat mode
    vtkNew<vtkTexture> texture;
    texture->SetInputData(image_data);
    texture->SetRepeat(1);  // Enable tiling
    texture->InterpolateOff();  // Sharp edges, no blurring
    texture->UseSRGBColorSpaceOn();
    
    // Set up texture coordinates for proper tiling
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(_plane_source->GetOutputPort());
    
    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(mapper);
    _vtk_actor->GetProperty()->SetBaseColorTexture(texture);
    _vtk_actor->SetUseBounds(false);    // ignore the plane in camera clipping computations

    // Calculate how many times to repeat the texture
    double repeat_count_x = _plane->width();
    double repeat_count_y = _plane->height();
    
    // Manually set texture coordinates for tiling
    vtkNew<vtkFloatArray> tcoords;
    tcoords->SetNumberOfComponents(2);
    tcoords->SetNumberOfTuples(4);
    tcoords->SetTuple2(0, 0, 0);
    tcoords->SetTuple2(1, repeat_count_x, 0);
    tcoords->SetTuple2(2, 0, repeat_count_y);
    tcoords->SetTuple2(3, repeat_count_x, repeat_count_y);
    tcoords->SetName("TextureCoordinates");
    
    _plane_source->Update();
    _plane_source->GetOutput()->GetPointData()->SetTCoords(tcoords);

    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

    // Transform the actor to match the plane's normal
    // Vec3r desired_normal = _plane->normal().normalized();
    // Vec3r default_normal(0, 0, 1);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // Vec3r axis = default_normal.cross(desired_normal);
    // double axis_length = axis.norm();

    // if (axis_length > 1e-6)
    // {
    //     axis /= axis_length;
    //     double angle = std::acos(default_normal.dot(desired_normal));
    //     double angle_degrees = angle * 180.0 / M_PI;
        
    //     _vtk_transform->RotateWXYZ(angle_degrees, axis[0], axis[1], axis[2]);
    // }
    // else if (desired_normal.dot(default_normal) < 0)
    // {
    //     _vtk_transform->RotateWXYZ(180.0, 1, 0, 0);
    // }

    // Also apply position offset if your plane has one
    // Vec3r position = _plane->position();
    // transform->Translate(position[0], position[1], position[2]);

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> plane_transform_mat = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>::Identity();
    plane_transform_mat.block<3,3>(0,0) = _plane->com().orientation;
    plane_transform_mat.block<3,1>(0,3) = _plane->com().position;
    _vtk_transform->SetMatrix(plane_transform_mat.data());

    _vtk_actor->SetUserTransform(_vtk_transform);

}

void PlaneGraphicsObject::update()
{
    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> plane_transform_mat = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>::Identity();
    plane_transform_mat.block<3,3>(0,0) = _plane->com().orientation;
    plane_transform_mat.block<3,1>(0,3) = _plane->com().position;
    _vtk_transform->SetMatrix(plane_transform_mat.data());
}

} // namespace Graphics