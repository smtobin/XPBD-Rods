#include "simobject/rod/CubicHermiteRodElement.hpp"

namespace SimObject
{

Real cubicHermiteN1(Real s_hat) { return 1 - 3*s_hat*s_hat + 2*s_hat*s_hat*s_hat; }
Real cubicHermiteN2(Real s_hat) { return s_hat*(1-s_hat)*(1-s_hat); }
Real cubicHermiteN3(Real s_hat) { return 3*s_hat*s_hat - 2*s_hat*s_hat*s_hat; }
Real cubicHermiteN4(Real s_hat) { return s_hat*s_hat*(s_hat-1); }
Real dcubicHermiteN1(Real s_hat) { return -6*s_hat + 6*s_hat*s_hat; }
Real dcubicHermiteN2(Real s_hat) { return 1-4*s_hat+3*s_hat*s_hat; }
Real dcubicHermiteN3(Real s_hat) { return 6*s_hat - 6*s_hat*s_hat; }
Real dcubicHermiteN4(Real s_hat) { return 3*s_hat*s_hat - 2*s_hat; }

CubicHermiteRodElement::CubicHermiteRodElement(
    const NodeArrayType& nodes_list, 
    const DerivativeArrayType& dp_ds, const DerivativeArrayType& dR_ds,
    Real rest_length
)
    : RodElement_Base(rest_length, Vec3r::Zero()), _nodes(nodes_list), _dp_ds(dp_ds), _dR_ds(dR_ds),
     _bases{cubicHermiteN1, cubicHermiteN2, cubicHermiteN3, cubicHermiteN4},
     _bases_derivatives{dcubicHermiteN1, dcubicHermiteN2, dcubicHermiteN3, dcubicHermiteN4}
{
}

std::array<Real, 4> CubicHermiteRodElement::lumpedMasses()
{
    // HRZ lumping (not used - the full mass matrix is used instead)
    return {156.0/320.0, 4.0/320.0, 156.0/320.0, 4.0/320.0};
}

Vec3r CubicHermiteRodElement::position(Real s_hat) const
{
    // std::cout << "s_hat: " << s_hat << std::endl;
    // std::cout << "basis[0] " << _bases[0](s_hat) << "  nodes[0] position: " << _nodes[0]->position.transpose() << std::endl;
    // std::cout << "basis[2] " << _bases[2](s_hat) << "  _dp_ds[0] position: " << _dp_ds[0]->position.transpose() << std::endl;
    // std::cout << "basis[1] " << _bases[1](s_hat) << "  nodes[1] position: " << _nodes[1]->position.transpose() << std::endl;
    // std::cout << "basis[3] " << _bases[3](s_hat) << "  _dp_ds[1] position: " << _dp_ds[1]->position.transpose() << std::endl;
    Vec3r p =  _bases[0](s_hat)*_nodes[0]->position + _bases[1](s_hat)*(_dp_ds[0]->position) + 
                _bases[2](s_hat)*_nodes[1]->position + _bases[3](s_hat)*(_dp_ds[1]->position);
    return p;
}

Vec3r CubicHermiteRodElement::previousPosition(Real s_hat) const
{
    Vec3r p =  _bases_derivatives[0](s_hat)*_nodes[0]->prev_position + _bases_derivatives[1](s_hat) * (_dp_ds[0]->prev_position) + 
                _bases_derivatives[2](s_hat)*_nodes[1]->prev_position + _bases_derivatives[3](s_hat) * (_dp_ds[1]->prev_position);
    return p;
}

Vec3r CubicHermiteRodElement::linearVelocity(Real s_hat) const
{
    throw std::runtime_error("Not implemented");

    return Vec3r::Zero();
}

Vec3r CubicHermiteRodElement::dposition_dshat(Real s_hat) const
{
    // std::cout << "s_hat: " << s_hat << std::endl;
    // std::cout << "basis'[0] " << _bases_derivatives[0](s_hat) << "  nodes[0] position: " << _nodes[0]->position.transpose() << std::endl;
    // std::cout << "basis'[2] " << _bases_derivatives[2](s_hat) << "  _dp_ds[0] position: " << _dp_ds[0]->position.transpose() << std::endl;
    // std::cout << "basis'[1] " << _bases_derivatives[1](s_hat) << "  nodes[1] position: " << _nodes[1]->position.transpose() << std::endl;
    // std::cout << "basis'[3] " << _bases_derivatives[3](s_hat) << "  _dp_ds[1] position: " << _dp_ds[1]->position.transpose() << std::endl;
    Vec3r dp =  _bases_derivatives[0](s_hat)*_nodes[0]->position + _bases_derivatives[1](s_hat)*(_dp_ds[0]->position) + 
                _bases_derivatives[2](s_hat)*_nodes[1]->position + _bases_derivatives[3](s_hat)*(_dp_ds[1]->position);

    // std::cout << "dp: " << dp.transpose() << std::endl;
    return dp;
}

Mat3r CubicHermiteRodElement::orientation(Real s_hat) const
{
    Vec3r R_diff = Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation);

    // use 4th order Magnus expansion to get orientation from curvature
    Real s1 = s_hat/2 - std::sqrt(3)*s_hat/6;
    Real s2 = s_hat/2 + std::sqrt(3)*s_hat/6;
    Vec3r u1 = _rest_length * bendingStrain(s1);
    Vec3r u2 = _rest_length * bendingStrain(s2);
    Vec3r theta = s_hat/2 * (u1 + u2) + std::sqrt(3)*s_hat*s_hat/12 * u1.cross(u2);


    // get interpolated relative rotation vector
    // Vec3r theta_old = _bases[1](s_hat)*(_dR_ds[0]->position) + _bases[2](s_hat)*R_diff + _bases[3](s_hat)*(_dR_ds[1]->position);

    // std::cout << "new theta: " << theta.transpose() << "\t old theta: " << theta_old.transpose() << "\t diff: " << (theta-theta_old).norm() << std::endl;

    // use exponential map to get interpolated rotation
    return Math::Plus_SO3(_nodes[0]->orientation, theta);
}

Mat3r CubicHermiteRodElement::previousOrientation(Real s_hat) const
{
    Vec3r R_diff = Math::Minus_SO3(_nodes[1]->prev_orientation, _nodes[0]->prev_orientation);

    // get interpolated relative rotation vector
    Vec3r theta = _bases_derivatives[1](s_hat) * (_dR_ds[0]->prev_position) + _bases_derivatives[2](s_hat)*R_diff + _bases_derivatives[3](s_hat) * (_dR_ds[1]->prev_position);

    // use exponential map to get interpolated rotation
    return Math::Plus_SO3(_nodes[0]->prev_orientation, theta);
}

Real CubicHermiteRodElement::Ni(int shape_func_index, Real s_hat) const
{
    return _bases[shape_func_index](s_hat);
}

Real CubicHermiteRodElement::dNi_dshat(int shape_func_index, Real s_hat) const
{
    return _bases_derivatives[shape_func_index](s_hat);
}

Vec3r CubicHermiteRodElement::shearStrain(Real s_hat) const
{
    Mat3r R = orientation(s_hat);

    Vec3r dp_dshat = dposition_dshat(s_hat);

    // std::cout << "p 0: " << _nodes[0]->position.transpose() << std::endl;
    // std::cout << "h*p' 0: " << _dp_ds[0]->position.transpose() << std::endl;
    // std::cout << "p 1: " << _nodes[1]->position.transpose() << std::endl;
    // std::cout << "h*p; 1: " << _dp_ds[1]->position.transpose() << std::endl;

    return R.transpose() * dshat_ds() * dp_dshat - Vec3r(0,0,1);
}

Vec3r CubicHermiteRodElement::bendingStrain(Real s_hat) const
{
    // std::cout << "CubicHermite bending strain -- _dR_ds[0]: " << _dR_ds[0]->position.transpose() << "\t_dR_ds[1]: " << _dR_ds[1]->position.transpose() << std::endl; 
    // std::cout << "   _R[0]: " << Math::Log_SO3(_nodes[0]->orientation).transpose() << "\t_R[1]: " << Math::Log_SO3(_nodes[1]->orientation).transpose() << std::endl;
    Vec3r R_diff = Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation);

    // Vec3r theta = _bases[1](s_hat)*(_dR_ds[0]->position) + _bases[2](s_hat)*R_diff + _bases[3](s_hat)*(_dR_ds[1]->position);
    // Vec3r dtheta_ds = dshat_ds() * (_bases_derivatives[1](s_hat)*(_dR_ds[0]->position) + _bases_derivatives[2](s_hat)*R_diff + _bases_derivatives[3](s_hat)*(_dR_ds[1]->position) );

    // Vec3r u = Math::ExpMap_RightJacobian(theta) * dtheta_ds;
    Vec3r u = dshat_ds() * (_bases_derivatives[1](s_hat)*_dR_ds[0]->position +  _bases_derivatives[2](s_hat)*R_diff + _bases_derivatives[3](s_hat)*_dR_ds[1]->position);
    return u;
}

Vec6r CubicHermiteRodElement::strain(Real s_hat) const
{
    Vec6r both;
    both.head<3>() = shearStrain(s_hat);
    both.tail<3>() = bendingStrain(s_hat);

    // std::cout << "strain: " << both.transpose() << std::endl;

    return both;
}

CubicHermiteRodElement::StrainGradientMatType CubicHermiteRodElement::strainGradient(Real s_hat) const
{
    StrainGradientMatType grad;
    
    Vec3r R_diff = Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation);
    Mat3r gam_inv = Math::ExpMap_InvRightJacobian(R_diff);

    // stores d theta(s) / d Ri for i = 1,...,Order+1
    std::array<Mat3r, 2> dtheta_dRi;
    std::array<Mat3r, 2> dtheta_dRprimei;

    // stores d/d Ri ( d theta(s) / ds) for i = 1,...Order+1
    std::array<Mat3r, 2> dthetaprime_dRi;
    std::array<Mat3r, 2> dthetaprime_dRprimei;

    // theta(s)
    Vec3r theta = _bases[1](s_hat)*(_dR_ds[0]->position) + _bases[2](s_hat)*R_diff + _bases[3](s_hat)*(_dR_ds[1]->position);
    Vec3r dtheta_ds = dshat_ds() * (_bases_derivatives[1](s_hat)*(_dR_ds[0]->position) + _bases_derivatives[2](s_hat)*R_diff + _bases_derivatives[3](s_hat)*(_dR_ds[1]->position) );

    dtheta_dRi[0] = -_bases[2](s_hat) * gam_inv.transpose();
    dtheta_dRi[1] = _bases[2](s_hat) * gam_inv;
    dtheta_dRprimei[0] = _bases[1](s_hat)*Mat3r::Identity();
    dtheta_dRprimei[1] = _bases[3](s_hat)*Mat3r::Identity();

    dthetaprime_dRi[0] = -dshat_ds() * _bases_derivatives[2](s_hat) * gam_inv.transpose();
    dthetaprime_dRi[1] = dshat_ds() * _bases_derivatives[2](s_hat) * gam_inv;
    dthetaprime_dRprimei[0] = dshat_ds() * _bases_derivatives[1](s_hat) * Mat3r::Identity();
    dthetaprime_dRprimei[1] = dshat_ds() * _bases_derivatives[3](s_hat) * Mat3r::Identity();


    /** Compute gradients of shear strain */

    // precompute useful quantities for gradients of shear strain
    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R = _nodes[0]->orientation * exp_theta;


    Vec3r dp_dshat = dshat_ds() * dposition_dshat(s_hat);
    Mat3r RT_dp_ds_R = R.transpose() * Math::Skew3(dp_dshat) * R;

    // gradients of shear strain

    // w.r.t position
    grad.template block<3,3>(0,0) = R.transpose() * dshat_ds() * _bases_derivatives[0](s_hat);
    grad.template block<3,3>(0,6) = R.transpose() * dshat_ds() * _bases_derivatives[2](s_hat);
    grad.template block<3,3>(0,12) = R.transpose() * dshat_ds() * _bases_derivatives[1](s_hat);
    grad.template block<3,3>(0,18) = R.transpose() * dshat_ds() * _bases_derivatives[3](s_hat);
    

    // w.r.t orientation
    grad.template block<3,3>(0,3) = RT_dp_ds_R * (exp_theta.transpose() + gam_theta * dtheta_dRi[0]);
    grad.template block<3,3>(0,9) = RT_dp_ds_R * gam_theta * dtheta_dRi[1];
    grad.template block<3,3>(0,15) = RT_dp_ds_R * gam_theta * dtheta_dRprimei[0];
    grad.template block<3,3>(0,21) = RT_dp_ds_R * gam_theta * dtheta_dRprimei[1];
    


    /** Compute gradients of bending strain */

    // gradients of bending strain
    // w.r.t position
    grad.template block<3,3>(3,0) = Mat3r::Zero();
    grad.template block<3,3>(3,6) = Mat3r::Zero();
    grad.template block<3,3>(3,12) = Mat3r::Zero();
    grad.template block<3,3>(3,18) = Mat3r::Zero();

    // w.r.t orientation
    // Mat3r dexpmap_contract_j = Math::DExpMap_RightJacobian_Contract_j(theta, dtheta_ds);
    // grad.template block<3,3>(3,3) = dexpmap_contract_j * dtheta_dRi[0] + gam_theta * dthetaprime_dRi[0];
    // grad.template block<3,3>(3,9) = dexpmap_contract_j * dtheta_dRi[1] + gam_theta * dthetaprime_dRi[1];
    // grad.template block<3,3>(3,15) = dexpmap_contract_j * dtheta_dRprimei[0] + gam_theta * dthetaprime_dRprimei[0];
    // grad.template block<3,3>(3,21) = dexpmap_contract_j * dtheta_dRprimei[1] + gam_theta * dthetaprime_dRprimei[1];
    grad.template block<3,3>(3, 3) = -dshat_ds() * _bases_derivatives[2](s_hat) * gam_inv.transpose();
    grad.template block<3,3>(3, 9) = dshat_ds() * _bases_derivatives[2](s_hat) * gam_inv;
    grad.template block<3,3>(3, 15) = dshat_ds() * _bases_derivatives[1](s_hat) * Mat3r::Identity();
    grad.template block<3,3>(3, 21) = dshat_ds() * _bases_derivatives[3](s_hat) * Mat3r::Identity();

    return grad;
}

} // namespace SimObject