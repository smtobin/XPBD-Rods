#include "simobject/rigidbody/XPBDRigidMesh.hpp"

namespace SimObject
{

XPBDRigidMesh::XPBDRigidMesh(const Config::XPBDRigidMeshConfig& config)
    : XPBDRigidBody_Base(config)
{
    // load mesh from file
    _mesh = Mesh::loadFromFile(config.filename());

    // compute mass properties
    auto [mass, center_of_mass, rot_inertia] = _mesh.massProperties(config.density());

    // translate mesh so that its mass center is at the origin
    _mesh.moveDelta(-center_of_mass);

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
    _com.orientation = R * _com.orientation;
    _com.prev_orientation = _com.orientation;
    
}

} // namespace SimObject