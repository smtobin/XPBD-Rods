#include "simobject/rigidbody/XPBDRigidMesh.hpp"
#include <filesystem>
namespace SimObject
{

XPBDRigidMesh::XPBDRigidMesh(const Config::XPBDRigidMeshConfig& config)
    : XPBDRigidBody_Base(config)
{
    // load mesh from file
    _mesh = Mesh::loadFromFile(config.filename());

    // scale the mesh according to input size
    _mesh.resize(config.scale()[0], config.scale()[1], config.scale()[2]);

    // compute mass properties
    auto [mass, center_of_mass, rot_inertia] = _mesh.massProperties(config.density());

    // translate mesh so that its mass center is at the origin
    _mesh.moveDelta(-center_of_mass);

    // compute orientation where rotational inertia is diagonal
    // Since I is symmetric, use SelfAdjointEigenSolver
    Eigen::SelfAdjointEigenSolver<Mat3r> solver(rot_inertia);

    if (solver.info() != Eigen::Success)
    {
        throw std::runtime_error("XPBDRigidMesh::XPBDRigidMesh rotational inertia not symmetric?");
    }

    // principal axes are the columns of the rotational matrix
    Mat3r R = solver.eigenvectors();

    // transform inertia tensor into principal-axis coordinates
    Mat3r D = R.transpose() * rot_inertia * R;
    _com.Ib = D.diagonal();

    std::cout << "Ib: " << _com.Ib.transpose() << std::endl;
    std::cout << R * rot_inertia * R.transpose() << std::endl;

    _mesh.applyRotation(R.transpose());

    AABB mesh_bbox = _mesh.boundingBox();
    _unoriented_size = mesh_bbox.max - mesh_bbox.min;

    // update the orientation to reflect the rotation required for the rotational inertia to be diagonal
    _com.orientation = R * _com.orientation;
    _com.prev_orientation = _com.orientation;

    // create collision geometry (if necessary)
    if (config.collisions())
    {
        int sdf_grid_size = 128;
        std::string sdf_filename = config.filename();
        std::replace(sdf_filename.begin(), sdf_filename.end(), '/', '-');
        std::replace(sdf_filename.begin(), sdf_filename.end(), '.', '_');
        sdf_filename = ".sdf/" + sdf_filename + "_" + std::to_string(sdf_grid_size) + "_" + 
            std::to_string(config.scale()[0]) + "x" + std::to_string(config.scale()[1]) + "x" + std::to_string(config.scale()[2]) + ".sdf";

        // check to see if cached SDF exists
        if (std::filesystem::exists(sdf_filename))
        {
            // load from cache file
            _sdf = Collision::MeshSDF(&_com, sdf_filename);
        }
        else
        {
            // create the SDF
            _sdf = Collision::MeshSDF(&_com, _mesh.vertices(), _mesh.faces(), sdf_grid_size);
            _sdf.writeToFile(sdf_filename);
        }
    }
        
    _com.mass = mass;

    std::cout << "Mesh bbox: " << mesh_bbox.min.transpose() << " to " << mesh_bbox.max.transpose() << std::endl;
    std::cout << "Unoriented size: " << _unoriented_size.transpose() << std::endl;
    AABB bbox = boundingBox();
    std::cout << "Oriented bbox: " << bbox.min.transpose() << " to " << bbox.max.transpose() << std::endl;
    
}

AABB XPBDRigidMesh::boundingBox() const
{
    Vec3r size = _com.orientation.cwiseAbs() * _unoriented_size;

    AABB bbox;
    bbox.min = _com.position - 0.5 * size;
    bbox.max = _com.position + 0.5 * size;

    return bbox;
}

} // namespace SimObject