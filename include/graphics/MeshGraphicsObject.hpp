#include "graphics/GraphicsObject.hpp"
#include "config/render/ObjectRenderConfig.hpp"

#include "common/Mesh.hpp"
#include "simobject/OrientedParticle.hpp"

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkExtractEdges.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

namespace Graphics
{

class MeshGraphicsObject : public GraphicsObject
{
public:
    explicit MeshGraphicsObject(const Mesh* mesh, const SimObject::OrientedParticle* com, const Config::ObjectRenderConfig& render_config);

    vtkSmartPointer<vtkActor> edgesActor() { return _edges_vtk_actor; }

private:
    /** The mesh geometry */
    const Mesh* _mesh;
    /** A particle that represents the location (position + orientation) of the mesh */
    const SimObject::OrientedParticle* _com;

    /** Actor  for drawing edges of the mesh */
    vtkSmartPointer<vtkActor> _edges_vtk_actor;

    // vtkSmartPointer<vtkPolyDataNormals> _normals_generator;
};

} // namespace Graphics