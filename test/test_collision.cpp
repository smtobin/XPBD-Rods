#include "common/common.hpp"
#include "collision/CollisionScene.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"

#include "graphics/BoxGraphicsObject.hpp"

#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkCylinderSource.h>
#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>

void visualize(const SimObject::XPBDRigidBox& box1, const SimObject::XPBDRigidBox& box2, 
    const std::vector<Vec3r>& box1_collision_points, const std::vector<Vec3r>& box2_collision_points)
{
    Vec3r box1_color(1.0, 0.0, 0.0);
    Vec3r box2_color(0.0, 0.0, 1.0);

    Config::ObjectRenderConfig box1_graphics_config(
        Config::ObjectRenderConfig::RenderType::PBR, 
        std::nullopt, std::nullopt, std::nullopt,
        0.0, 0.5, 1.0,
        box1_color, false, true, false, false
    );

    Config::ObjectRenderConfig box2_graphics_config(
        Config::ObjectRenderConfig::RenderType::PBR, 
        std::nullopt, std::nullopt, std::nullopt,
        0.0, 0.5, 1.0,
        box2_color, false, true, false, false
    );

    Graphics::BoxGraphicsObject box1_graphics_obj(&box1, box1_graphics_config);
    Graphics::BoxGraphicsObject box2_graphics_obj(&box2, box2_graphics_config);
    vtkNew<vtkOpenGLRenderer> renderer;
    renderer->SetBackground(0.2, 0.2, 0.2);
    renderer->AddActor(box1_graphics_obj.actor());
    renderer->AddActor(box2_graphics_obj.actor());


    // Create points
    vtkSmartPointer<vtkPoints> points1 = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();

    for (unsigned i = 0; i < box1_collision_points.size(); i++)
    {
        points1->InsertNextPoint(box1_collision_points[i][0], box1_collision_points[i][1], box1_collision_points[i][2]);
        points2->InsertNextPoint(box2_collision_points[i][0], box2_collision_points[i][1], box2_collision_points[i][2]);
    }

    // Create polydata to hold the points
    vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
    polydata1->SetPoints(points1);
    vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
    polydata2->SetPoints(points2);

    // Create vertex glyphs to actually display the points
    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter1 = 
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexFilter1->SetInputData(polydata1);
    vertexFilter1->Update();

    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter2 = 
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexFilter2->SetInputData(polydata2);
    vertexFilter2->Update();

    // Map the points to graphics primitives
    vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper1->SetInputConnection(vertexFilter1->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper2->SetInputConnection(vertexFilter2->GetOutputPort());

    // Create an actor
    vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
    actor1->SetMapper(mapper1);
    actor1->GetProperty()->SetPointSize(10);  // Make points larger
    actor1->GetProperty()->SetColor(box1_color[0], box1_color[1], box1_color[2]);

    vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
    actor2->SetMapper(mapper2);
    actor2->GetProperty()->SetPointSize(10);  // Make points larger
    actor2->GetProperty()->SetColor(box2_color[0], box2_color[1], box2_color[2]);

    renderer->AddActor(actor1);
    renderer->AddActor(actor2);

    vtkNew<vtkRenderWindow> render_window;
    render_window->AddRenderer(renderer);
    render_window->SetSize(600,600);

    vtkNew<vtkRenderWindowInteractor> interactor;
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor->SetInteractorStyle(style);
    interactor->SetRenderWindow(render_window);
    render_window->Render();

    interactor->Start();
}

int main()
{
    Collision::CollisionScene scene;

    Vec3r pos1(0,0,0);
    Vec3r rot1(0,0,45);  // XYZ Euler angle convention (in degrees)
    Vec3r size1(1.2,1.8,1);
    Config::XPBDRigidBoxConfig box1_config("box1", pos1, rot1, Vec3r(0,0,0), Vec3r(0,0,0), 1000, size1);

    Vec3r pos2(0.4, 1.1, 0.4);
    Vec3r rot2(45,60,0);
    Vec3r size2(0.4,1,1.2);
    Config::XPBDRigidBoxConfig box2_config("box2", pos2, rot2, Vec3r(0,0,0), Vec3r(0,0,0), 1000, size2);

    SimObject::XPBDRigidBox box1(box1_config);
    SimObject::XPBDRigidBox box2(box2_config);

    scene.addObject(&box1);
    scene.addObject(&box2);

    const std::vector<Collision::DetectedCollision>& detected_collisions = scene.detectCollisions();
    std::vector<Vec3r> box1_collision_points;
    std::vector<Vec3r> box2_collision_points;
    for (const auto& collision : detected_collisions)
    {
        std::cout << "\n=== Collision between " << collision.particle1 << " and " << collision.particle2 << " ===" << std::endl;
        std::cout << "Position1: " << collision.particle1->position.transpose() << std::endl;
        std::cout << "Position2: " << collision.particle2->position.transpose() << std::endl;
        std::cout << "Contact point 1 (local): " << collision.cp_local1.transpose() << std::endl;
        std::cout << "Contact point 2 (local): " << collision.cp_local2.transpose() << std::endl;
        Vec3r cp1_global = (collision.particle1->orientation * collision.cp_local1 + collision.particle1->position);
        Vec3r cp2_global = (collision.particle2->orientation * collision.cp_local2 + collision.particle2->position);
        std::cout << "Contact point 1 (global): " << cp1_global.transpose() << std::endl;
        std::cout << "Contact point 2 (global): " << cp2_global.transpose() << std::endl;
        std::cout << "Collision normal: " << collision.normal.transpose() << std::endl;

        if (collision.particle1 == &box1.com())
        {
            box1_collision_points.push_back(cp1_global);
            box2_collision_points.push_back(cp2_global);
        }
        else
        {
            box2_collision_points.push_back(cp1_global);
            box1_collision_points.push_back(cp2_global);
        }

        std::cout << "(cp2 - cp1).dot(normal): " << (cp2_global - cp1_global).dot(collision.normal) << std::endl;
    }


    /** Visualize boxes */

    visualize(box1, box2, box1_collision_points, box2_collision_points);
}