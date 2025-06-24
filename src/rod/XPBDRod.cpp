#include "rod/XPBDRod.hpp"
#include "common/math.hpp"

#include <iostream>

namespace Rod
{

void XPBDRod::setup()
{
    _dl = _length / (_num_nodes - 1);
    // std::cout << "_dl: " << _dl << std::endl;

    // compute mass/inertia properties
    _m_total = _length * _cross_section->area() * _density;
    _m_node = _m_total / _num_nodes;
    _I_rot = _m_node / _cross_section->area() * Vec3r(_cross_section->Ix(), _cross_section->Iy(), _cross_section->Iz());
    _I_rot_inv = 1.0/_I_rot.array();

    // std::cout << "I_rot:\n" << _I_rot << std::endl;
    // std::cout << "I_rot_inv:\n" << _I_rot_inv << std::endl;

    _inertia_mat_inv.conservativeResize(6*_num_nodes);
    for (int i = 0; i < _num_nodes; i++)
    {
        _inertia_mat_inv.block<3,1>(6*i,0) = 1.0/_m_node * Vec3r::Ones();
        _inertia_mat_inv.block<3,1>(6*i+3,0) = _I_rot_inv;
    }

    _alpha.conservativeResize(6*_num_nodes);
    _alpha.block<6,1>(0,0) = Vec6r::Zero();
    for (int i = 1; i < _num_nodes; i++)
    {
        _alpha.block<6,1>(6*i,0) = 1/_dl * Vec6r( 1.0 / (_G * _cross_section->area()), 1.0 / (_G * _cross_section->area()), 1.0 / (_E * _cross_section->area()),
                                                  1.0 / (_E * _cross_section->Ix()),   1.0 / (_E * _cross_section->Iy()),   1.0 / (_G * _cross_section->Iz()) );
    }

    // reserve space for constraints and constraint gradients
    _C_vec.conservativeResize(6*_num_nodes);
    _delC_mat.conservativeResize(6*_num_nodes, 6*_num_nodes);
    _LHS_mat.conservativeResize(6*_num_nodes, 6*_num_nodes);
    _lambda.conservativeResize(6*_num_nodes);
    _dlam.conservativeResize(6*_num_nodes);
    _dx.conservativeResize(6*_num_nodes);
    
}

void XPBDRod::update(Real dt, Real g_accel)
{
    _lambda = VecXr::Zero(6*_num_nodes);
    _prev_nodes = _nodes;
    _inertialUpdate(dt, g_accel);

    for (int gi = 0; gi < 1; gi++)
    {
        _computeConstraintVec();
        _computeConstraintGradients();

        // std::cout << "Constraint vector:\n" << _C_vec << std::endl;
        // std::cout << "Constraint gradients:\n" << _delC_mat << std::endl;

        _LHS_mat = _delC_mat * _inertia_mat_inv.asDiagonal() * _delC_mat.transpose();
        _LHS_mat += (_alpha / (dt*dt)).asDiagonal();
        // std::cout << "LHS:\n" << _LHS_mat << std::endl;
        _RHS_vec = -_C_vec - (_alpha / (dt*dt)).asDiagonal() * _lambda;
        _dlam = _LHS_mat.llt().solve(_RHS_vec);

        // std::cout << "dlam:\n" << _dlam << std::endl;
        _dx = _inertia_mat_inv.asDiagonal() * _delC_mat.transpose() * _dlam;
        // std::cout << "dx:\n" << _dx << std::endl;
        _positionUpdate(); 

        _lambda += _dlam;
    }


    _velocityUpdate(dt);
}

void XPBDRod::_inertialUpdate(Real dt, Real g_accel)
{
    for (int i = 0; i < _num_nodes; i++)
    {
        auto& node = _nodes[i];

        // position inertial update
        Vec3r F_ext = _m_node * Vec3r(0,-g_accel,0);
        if (i==_num_nodes-1)
            F_ext = Vec3r(10000,0,0);
        node.position += dt*node.velocity + dt*dt/_m_node*F_ext;

        // orientation inertial update
        Vec3r T_ext = Vec3r(0,0,0);
        // if (i==_num_nodes - 1)
        //     T_ext = Vec3r(0,0,10000);
        Vec3r so3_update = dt*node.ang_velocity + dt*dt*_I_rot_inv.asDiagonal()*(T_ext - node.ang_velocity.cross(_I_rot.asDiagonal()*node.ang_velocity));
        // std::cout << "ang velocity:\n" << node.ang_velocity << std::endl;
        // std::cout << "so3_update:\n" << so3_update << std::endl;
        node.orientation = Plus_SO3(node.orientation, so3_update);

        // std::cout << "Node " << i << " position:\n" << node.position << std::endl;
        // std::cout << "Node " << i << " orientation:\n" << node.orientation << std::endl;
    }
}

void XPBDRod::_computeConstraintVec()
{
    // fixed base constraint
    _C_vec( Eigen::seqN(0,3) ) = _nodes[0].position - _prev_nodes[0].position;
    _C_vec( Eigen::seqN(3,3) ) = Minus_SO3(_nodes[0].orientation, _prev_nodes[0].orientation);

    // elastic rod constraints
    for (int ci = 1; ci < _num_nodes; ci++)
    {
        // computing the constraints for the (ci)th segment, which involves nodes (ci-1) and ci
        const Vec3r dp = _nodes[ci].position - _nodes[ci-1].position;
        // std::cout << "dp:\n" << dp << std::endl;
        const Vec3r C_v = (_nodes[ci].orientation.transpose() + _nodes[ci-1].orientation.transpose()) * 0.5 * dp / _dl - Vec3r(0,0,1);
        const Vec3r C_u = Log_SO3(_nodes[ci-1].orientation.transpose() * _nodes[ci].orientation) / _dl;

        // std::cout << "Cv_" << ci << ":\n" << C_v << std::endl;
        _C_vec( Eigen::seqN(6*ci,3) ) = C_v;
        _C_vec( Eigen::seqN(6*ci+3,3) ) = C_u;
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
        const Mat3r dCv_dp_iminus1 = -0.5*(_nodes[ci-1].orientation.transpose() + _nodes[ci].orientation.transpose()) / _dl;
        const Mat3r dCv_dp_i = -dCv_dp_iminus1;
        
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

void XPBDRod::_positionUpdate()
{
    for (int i = 0; i < _num_nodes; i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*i,3) );
        const Vec3r dor = _dx( Eigen::seqN(6*i+3,3) );
        _nodes[i].position += dp;
        _nodes[i].orientation = Plus_SO3(_nodes[i].orientation, dor);

        // std::cout << "New orientation " << i << ":\n" << _nodes[i].orientation << std::endl;
    }
}

void XPBDRod::_velocityUpdate(Real dt)
{
    for (unsigned i = 0; i < _nodes.size(); i++)
    {
        _nodes[i].velocity = (_nodes[i].position - _prev_nodes[i].position)/dt;
        // std::cout << "Orientation " << i << ":\n" << _nodes[i].orientation << std::endl;
        // std::cout << "Prev orientation " << i << ":\n" << _prev_nodes[i].orientation << std::endl;
        // std::cout << "Orientation velocity update Minus_SO3:\n" << Minus_SO3(_nodes[i].orientation, _prev_nodes[i].orientation) << std::endl;
        _nodes[i].ang_velocity = Minus_SO3(_nodes[i].orientation, _prev_nodes[i].orientation)/dt;
    }
}

} // namespace Rod