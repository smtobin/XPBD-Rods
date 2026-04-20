#include "graphics/GraphicsObject.hpp"
#include "config/ObjectRenderConfig.hpp"
#include "config/MeshRenderConfig.hpp"

#include "common/Mesh.hpp"
#include "simobject/OrientedParticle.hpp"

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkExtractEdges.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkTransform.h>

namespace Graphics
{

class MeshGraphicsObject : public GraphicsObject
{
public:
    explicit MeshGraphicsObject(const Mesh* mesh, const SimObject::OrientedParticle* com, const Config::MeshRenderConfig& render_config);

    vtkSmartPointer<vtkActor> edgesActor() { return _edges_vtk_actor; }

    virtual void update() override;

private:
    /** The mesh geometry */
    const Mesh* _mesh;
    /** A particle that represents the location (position + orientation) of the mesh */
    const SimObject::OrientedParticle* _com;

    /** Actor  for drawing edges of the mesh */
    vtkSmartPointer<vtkActor> _edges_vtk_actor;

    /** Transform for the mesh COM */
    vtkSmartPointer<vtkTransform> _vtk_transform;

    // vtkSmartPointer<vtkPolyDataNormals> _normals_generator;
};

} // namespace Graphics