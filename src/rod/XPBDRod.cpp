#include "rod/XPBDRod.hpp"
#include "common/math.hpp"

namespace Rod
{

void XPBDRod::setup()
{
    _dl = _length / (_num_nodes - 1);

    // compute mass/inertia properties
    _m_total = _length * _cross_section->area() * _density;
    _m_node = _m_total / _num_nodes;
    _I_rot << _cross_section->Ix(), _cross_section->Iy(), _cross_section->J();
    _I_rot_inv << 1.0/_cross_section->Ix(), 1.0/_cross_section->Iy(), 1.0/_cross_section->J();

    // reserve space for constraints and constraint gradients
    _C_vec.conservativeResize(6*_num_nodes);
    _delC_mat.conservativeResize(6*_num_nodes, 6*_num_nodes);
}

void XPBDRod::update(Real dt, Real g_accel)
{
    _intertialUpdate(dt, g_accel);


    for (int gi = 0; gi < 1; gi++)
    {
        _computeConstraintVec();
        _computeConstraintGradients();

        // TODO: solve system for delta lambda and update positions
    }


    _velocityUpdate(dt);
}

void XPBDRod::_inertialUpdate(Real dt, Real g_accel)
{
    for (auto& node : _nodes)
    {
        // position inertial update
        Vec3r F_ext = _m_node * g_accel;
        node.position += dt*node.velocity + dt*dt/_m_node*F_ext;

        // orientation inertial update
        Vec3r T_ext = Vec3r(0,0,0);
        Vec3r so3_update = dt*node.ang_velocity + dt*dt*_I_rot_inv*(T_ext - node.ang_velocity.cross(_I_rot*node.ang_velocity));
        node.orientation = Plus_SO3(node.orientation, so3_update);
    }
}

void XPBDRod::_computeConstraintVec()
{
    // fixed base constraint
    _C_vec.block<3,1>(0,0) = _nodes[0].position - _prev_nodes[0].position;
    _C_vec.block<3,1>(3,0) = Minus_SO3(_nodes[0].orientation, _prev_nodes[0].orientation);

    // elastic rod constraints
    for (int ci = 1; ci < _num_nodes; ci++)
    {
        // computing the constraints for the (ci)th segment, which involves nodes (ci-1) and ci
        const Vec3r dp = _nodes[ci].position - _nodes[ci-1].position;
        const Vec3r C_v = (_nodes[ci].orientation + _nodes[ci-1].orientation) * dp / (2*_dl) - Vec3r(0,0,1);
        const Vec3r C_u = Log_SO3(_nodes[ci-1].orientation.transpose() * _nodes[ci].orientation) / _dl;

        _C_vec.block<3,1>(6*ci,0) = C_v;
        _C_vec.block<3,1>(6*ci + 3,0) = C_u;
    }
}

void XPBDRod::_computeConstraintGradients()
{
    _delC_mat = MatXr::Zero(6*_num_nodes, 6*_num_nodes);

    // fixed base constraint gradient
    const Vec3r dtheta0 = Minus_SO3(_nodes[0].orientation, _prev_nodes[0].orientation);
    const Mat3r jac_inv0 = ExpMap_Jacobian(dtheta0).inverse();
    _delC_mat.block<3,3>(0,0) = Mat3r::Identity();
    _delC_mat.block<3,3>(3,3) = jac_inv0;

    // elastic rod constraints
    for (int ci = 1; ci < _num_nodes; ci++)
    {
        // computing the constraint gradients for the (ci)th segment, which involves nodes (ci-1) and ci
        const Mat3r dCv_dp_iminus1 = -_nodes[ci-1].orientation.transpose() / _dl;
        const Mat3r dCv_dp_i = _nodes[ci].orientation.transpose() / _dl;
        
        const Vec3r dp = _nodes[ci].position - _nodes[ci-1].position;
        const Mat3r dCv_dor_iminus1 = 0.5*Skew3(_nodes[ci-1].orientation.transpose() * dp / _dl);
        const Mat3r dCv_dor_i = 0.5*Skew3(_nodes[ci].orientation.transpose() * dp / _dl);

        const Vec3r dtheta = Log_SO3(_nodes[ci-1].orientation.transpose() * _nodes[ci].orientation);
        const Mat3r jac_inv = ExpMap_Jacobian(dtheta).inverse();
        const Mat3r dCu_dor_iminus1 = -jac_inv / _dl;
        const Mat3r dCu_dor_i = jac_inv / _dl;

        // submatrices in correct spot in overall delC matrix
        _delC_mat.block<3,3>(6*ci, 6*(ci-1)) = dCv_dp_iminus1;
        _delC_mat.block<3,3>(6*ci, 6*(ci-1)+3) = dCv_dor_iminus1;
        _delC_mat.block<3,3>(6*ci, 6*ci) = dCv_dp_i;
        _delC_mat.block<3,3>(6*ci, 6*ci+3) = dCv_dor_i;
        _delC_mat.block<3,3>(6*ci+3, 6*(ci-1)+3) = dCu_dor_iminus1;
        _delC_mat.block<3,3>(6*ci+3, 6*ci+3) = dCu_dor_i;
    }
}

void XPBDRod::_velocityUpdate(Real dt)
{
    for (unsigned i = 0; i < _nodes.size(); i++)
    {
        _nodes[i].velocity = (_nodes[i].position - _prev_nodes[i].position)/dt;
        _nodes[i].ang_velocity = Minus_SO3(_nodes[i].orientation, _prev_nodes[i].orientation)/dt;
    }
}

} // namespace Rod