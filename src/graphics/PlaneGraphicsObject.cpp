#include "graphics/PlaneGraphicsObject.hpp"

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
    
    // Set up texture coordinates for proper tiling
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(_plane_source->GetOutputPort());
    
    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(mapper);
    _vtk_actor->SetTexture(texture);
    
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

}

void PlaneGraphicsObject::update()
{
    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    // Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> sphere_transform_mat = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>::Identity();
    // sphere_transform_mat.block<3,3>(0,0) = _sphere->com().orientation;
    // sphere_transform_mat.block<3,1>(0,3) = _sphere->com().position;
    // _vtk_transform->SetMatrix(sphere_transform_mat.data());
}

} // namespace Graphics