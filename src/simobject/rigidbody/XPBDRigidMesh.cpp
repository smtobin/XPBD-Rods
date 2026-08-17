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

    // update the orientation to reflect the rotation required for the rotational inertia to be diagonal
    // _com.orientation = R * _com.orientation;
    // _com.prev_orientation = _com.orientation;

    // compute the unoriented AABB size of the mesh for computing the updated AABB later
    SimObject::AABB aabb = _mesh.boundingBox();
    Vec3r size = aabb.max - aabb.min;
    // rotate the bounding box back to what the XPBDRigidMesh thinks is 0 rotation
    Mat3r RT_abs = R.transpose().cwiseAbs();
    _unoriented_size = RT_abs.col(0) * size[0] + RT_abs.col(1) * size[1] + RT_abs.col(2) * size[2];
    
}

AABB XPBDRigidMesh::boundingBox() const
{
    Mat3r R_abs = _com.orientation.cwiseAbs();
    Vec3r AABB_size = R_abs.col(0) * _unoriented_size[0] + R_abs.col(1) * _unoriented_size[1] + R_abs.col(2) * _unoriented_size[2];
    AABB bbox;
    bbox.min = _com.position - AABB_size/2;
    bbox.max = _com.position + AABB_size/2;
    return bbox;
}

} // namespace SimObject