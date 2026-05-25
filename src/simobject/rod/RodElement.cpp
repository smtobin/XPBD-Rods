#include "simobject/rod/RodElement.hpp"

namespace SimObject
{

Real linearN1(Real s_hat) { return -s_hat + 1; }
Real linearN2(Real s_hat) { return s_hat; }
Real dlinearN1(Real /* s_hat */) { return -1; }
Real dlinearN2(Real /* s_hat */) { return 1; }
// Real d2linearN1(Real /* s_hat */) { return 0; }
// Real d2linearN2(Real /* s_hat */) { return 0; }

Real quadraticN1(Real s_hat) { return 2*s_hat*s_hat - 3*s_hat + 1; }
Real quadraticN2(Real s_hat) { return -4*s_hat*s_hat + 4*s_hat; }
Real quadraticN3(Real s_hat) { return 2*s_hat*s_hat - s_hat; }
Real dquadraticN1(Real s_hat) { return 4*s_hat - 3; }
Real dquadraticN2(Real s_hat) { return -8*s_hat + 4; }
Real dquadraticN3(Real s_hat) { return 4*s_hat - 1; }
// Real d2quadraticN1(Real /* s_hat */) { return 4; }
// Real d2quadraticN2(Real /* s_hat */) { return -8; }
// Real d2quadraticN3(Real /* s_hat */) { return 4; }

Real cubicN1(Real s_hat) { return -9.0/2.0 * (s_hat - 1.0/3.0) * (s_hat - 2.0/3.0) * (s_hat - 1); }
Real cubicN2(Real s_hat) { return 27.0/2.0 * s_hat * (s_hat - 2.0/3.0) * (s_hat - 1); }
Real cubicN3(Real s_hat) { return -27.0/2.0 * s_hat * (s_hat - 1.0/3.0) * (s_hat - 1); }
Real cubicN4(Real s_hat) { return 9.0/2.0 * s_hat * (s_hat - 1.0/3.0) * (s_hat - 2.0/3.0); }
Real dcubicN1(Real s_hat) { return -27.0/2.0*s_hat*s_hat + 18*s_hat - 11.0/2.0; }
Real dcubicN2(Real s_hat) { return 81.0/2.0*s_hat*s_hat - 45*s_hat + 9; }
Real dcubicN3(Real s_hat) { return -81.0/2.0*s_hat*s_hat + 36*s_hat - 9.0/2.0; }
Real dcubicN4(Real s_hat) { return 27.0/2.0*s_hat*s_hat - 9*s_hat + 1; }

template<int Order>
RodElement<Order>::RodElement(const NodeArrayType& nodes_list, Real rest_length, const Vec3r& curvature)
    : RodElement_Base(rest_length, curvature), _nodes(nodes_list), _bases{}, _bases_derivatives{}
{
    // get pointers to basis functions and their derivatives, according to element order
    if constexpr (Order == 0)
    {
        // do nothing - order 0 corresponds to "rigid body" elements that don't use shape functions
    }
    else if constexpr (Order == 1)
    {
        _bases = {linearN1, linearN2};
        _bases_derivatives = {dlinearN1, dlinearN2};
    }
    else if constexpr (Order == 2)
    {
        _bases = {quadraticN1, quadraticN2, quadraticN3};
        _bases_derivatives = {dquadraticN1, dquadraticN2, dquadraticN3};
    }
    else if constexpr (Order == 3)
    {
        _bases = {cubicN1, cubicN2, cubicN3, cubicN4};
        _bases_derivatives = {dcubicN1, dcubicN2, dcubicN3, dcubicN4};
    }
    else
    {
        static_assert(0);
    }
}

template<int Order>
std::array<Real, RodElement<Order>::NumNodes> RodElement<Order>::lumpedMasses()
{
    if constexpr (Order == 0)
    {
        return {0.5, 0.5};
    }
    else if constexpr (Order == 1)
    {
        return {0.5, 0.5};
    }
    else if constexpr (Order == 2)
    {
        return {1.0/6.0, 2.0/3.0, 1.0/6.0};
    }
    else if constexpr (Order == 3)
    {
        return {1.0/8.0, 3.0/8.0, 3.0/8.0, 1.0/8.0};
    }
    else
    {
        static_assert(0);
        return std::array<Real, Order+1>{};
    }
}

template<int Order>
Vec3r RodElement<Order>::position(Real s_hat) const
{
    Vec3r p = _bases[0](s_hat) * _nodes[0]->position;
    for (int i = 1; i < Order+1; i++)
    {
        p += _bases[i](s_hat) * _nodes[i]->position;
    }

    return p;
}

template<int Order>
Vec3r RodElement<Order>::previousPosition(Real s_hat) const
{
    Vec3r p = _bases[0](s_hat) * _nodes[0]->prev_position;
    for (int i = 1; i < Order+1; i++)
    {
        p += _bases[i](s_hat) * _nodes[i]->prev_position;
    }

    return p;
}

template<int Order>
Vec3r RodElement<Order>::dposition_dshat(Real s_hat) const
{
    Vec3r dp = _bases_derivatives[0](s_hat) * _nodes[0]->position;
    for (int i = 1; i < Order+1; i++)
    {
        dp += _bases_derivatives[i](s_hat) * _nodes[i]->position;
    }

    return dp;
}

template <int Order>
Mat3r RodElement<Order>::orientation(Real s_hat) const
{
    // get interpolated relative rotation vector
    Vec3r theta = Vec3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        theta += _bases[i](s_hat) * Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
    }

    Vec3r R2_min_R1 = Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation);
    // if (R2_min_R1.norm() > 1)
    // {
    //     std::cout << "R2_min_R1 close to PI! Mag: " << R2_min_R1.norm() << std::endl;
    // }

    // use exponential map to get interpolated rotation
    return Math::Plus_SO3(_nodes[0]->orientation, theta);
}

template <int Order>
Mat3r RodElement<Order>::previousOrientation(Real s_hat) const
{
    // get interpolated relative rotation vector
    Vec3r theta = Vec3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        theta += _bases[i](s_hat) * Math::Minus_SO3(_nodes[i]->prev_orientation, _nodes[0]->prev_orientation);
    }

    // use exponential map to get interpolated rotation
    return Math::Plus_SO3(_nodes[0]->prev_orientation, theta);
}

template <int Order>
Real RodElement<Order>::Ni(int shape_func_index, Real s_hat) const
{
    return _bases[shape_func_index](s_hat);
}

template <int Order>
Real RodElement<Order>::dNi_dshat(int shape_func_index, Real s_hat) const
{
    return _bases_derivatives[shape_func_index](s_hat);
}

template <int Order>
Vec3r RodElement<Order>::shearStrain(Real s_hat) const
{
    Mat3r R = orientation(s_hat);

    Vec3r dp_ds = Vec3r::Zero();
    for (int i = 0; i < Order+1; i++)
    {
        dp_ds += _bases_derivatives[i](s_hat) * _nodes[i]->position;
    }

    return R.transpose() * dshat_ds() * dp_ds - Vec3r(0,0,1);
}

template <>
Vec3r RodElement<1>::shearStrain(Real s_hat) const
{
    Mat3r R = Math::Plus_SO3(_nodes[0]->orientation, s_hat * Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation));

    return 1/_rest_length * R.transpose() * (_nodes[1]->position - _nodes[0]->position) - Vec3r(0,0,1);
}

template <int Order>
Vec3r RodElement<Order>::bendingStrain(Real s_hat) const
{
    Vec3r theta = Vec3r::Zero();
    Vec3r dtheta_ds = Vec3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        theta += _bases[i](s_hat) * Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
        dtheta_ds += dshat_ds() * _bases_derivatives[i](s_hat) * Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
    }

    Vec3r u1 = Math::ExpMap_RightJacobian(theta) * dtheta_ds - _curvature;

    return u1;
}

template <>
Vec3r RodElement<1>::bendingStrain(Real /* s_hat */) const
{
    return 1.0/_rest_length * Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation) - _curvature;
}

template <int Order>
Vec6r RodElement<Order>::strain(Real s_hat) const
{
    Vec6r both;
    both.head<3>() = shearStrain(s_hat);
    both.tail<3>() = bendingStrain(s_hat);

    return both;
}

template <int Order>
typename RodElement<Order>::StrainGradientMatType RodElement<Order>::strainGradient(Real s_hat) const
{
    StrainGradientMatType grad;
    
    // stores (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Vec3r, Order+1> Ri_minus_R1;
    // stores Gamma^{-1} (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Mat3r, Order+1> gam_inv_Ri_minus_R1;
    // stores d theta(s) / d Ri for i = 1,...,Order+1
    std::array<Mat3r, Order+1> dtheta_dRi;
    // stores d/d Ri ( d theta(s) / ds) for i = 1,...Order+1
    std::array<Mat3r, Order+1> dtheta_ds_dRi;

    // theta(s)
    Vec3r theta = Vec3r::Zero();
    Vec3r dtheta_ds = Vec3r::Zero();

    // precompute quantities
    dtheta_dRi[0] = Mat3r::Zero();
    dtheta_ds_dRi[0] = Mat3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        Ri_minus_R1[i] = Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
        gam_inv_Ri_minus_R1[i] = Math::ExpMap_InvRightJacobian(Ri_minus_R1[i]);
        dtheta_dRi[i] = _bases[i](s_hat) * gam_inv_Ri_minus_R1[i];
        dtheta_ds_dRi[i] = dshat_ds() * _bases_derivatives[i](s_hat) * gam_inv_Ri_minus_R1[i];

        theta += _bases[i](s_hat) * Ri_minus_R1[i];
        dtheta_ds += dshat_ds() * _bases_derivatives[i](s_hat) * Ri_minus_R1[i];

        // add contribution to dtheta_dR0 and dtheta_ds_dR0
        dtheta_dRi[0] -= dtheta_dRi[i].transpose();
        dtheta_ds_dRi[0] -= dtheta_ds_dRi[i].transpose();
    }


    /** Compute gradients of shear strain */

    // precompute useful quantities for gradients of shear strain
    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R = _nodes[0]->orientation * exp_theta;


    Vec3r dp_ds = Vec3r::Zero();
    for (int i = 0; i < Order+1; i++)
    {
        dp_ds += dshat_ds() * _bases_derivatives[i](s_hat) * _nodes[i]->position;
    }
    Mat3r RT_dp_ds_R = R.transpose() * Math::Skew3(dp_ds) * R;

    // gradients of shear strain
    for (int i = 0; i < Order+1; i++)
    {
        grad.template block<3,3>(0,6*i) = R.transpose() * dshat_ds() * _bases_derivatives[i](s_hat);

        if (i == 0)
        {
            grad.template block<3,3>(0,6*i+3) = RT_dp_ds_R * (exp_theta.transpose() + gam_theta * dtheta_dRi[i]);
        }
        else
        {
            grad.template block<3,3>(0,6*i+3) = RT_dp_ds_R * gam_theta * dtheta_dRi[i];
        }
    }


    /** Compute gradients of bending strain */

    // gradients of bending strain
    // Mat3r dexpmap_contract_j = Math::DExpMap_RightJacobian_Contract_j(theta, dtheta_ds);
    Mat3r dexpmap_contract_j_approx = Math::DExpMap_RightJacobian_Contract_j_approx(theta, dtheta_ds);

    // std::cout << "||theta||: " << theta.norm() << std::endl;
    // std::cout << "dexpmap_contract_j full:\n" << dexpmap_contract_j << std::endl;
    // std::cout << "dexpmap_contract_j approx:\n" << dexpmap_contract_j_approx << std::endl;

    for (int i = 0; i < Order+1; i++)
    {
        // gradient w.r.t. positions = 0
        grad.template block<3,3>(3, 6*i) = Mat3r::Zero();

        // gradient w.r.t. orientation
        grad.template block<3,3>(3, 6*i+3) = dexpmap_contract_j_approx * dtheta_dRi[i] + gam_theta * dtheta_ds_dRi[i];    
    }

    return grad;
}

template <>
typename RodElement<1>::StrainGradientMatType RodElement<1>::strainGradient(Real s_hat) const
{
    StrainGradientMatType grad;
    
    Real inv_length = 1.0/_rest_length;

    // stores d theta(s) / d Ri for i = 1,...,Order+1
    std::array<Mat3r, 2> dtheta_dRi;
    // stores d/d Ri ( d theta(s) / ds) for i = 1,...Order+1
    std::array<Mat3r, 2> dtheta_ds_dRi;

    Vec3r R2_minus_R1 = Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation);
    Mat3r gam_inv = Math::ExpMap_InvRightJacobian(R2_minus_R1);

    // theta(s)
    Vec3r theta = s_hat * R2_minus_R1;

    // precompute quantities
    dtheta_dRi[1] = s_hat * gam_inv;
    dtheta_dRi[0] = -dtheta_dRi[1].transpose();

    dtheta_ds_dRi[1] = inv_length * gam_inv;
    dtheta_ds_dRi[0] = -dtheta_ds_dRi[1].transpose();


    /** Compute gradients of shear strain */

    // precompute useful quantities for gradients of shear strain
    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R = _nodes[0]->orientation * exp_theta;


    Vec3r dp_ds = 1/_rest_length * (_nodes[1]->position - _nodes[0]->position);
    Mat3r RT_dp_ds_R = Math::Skew3(R.transpose() * dp_ds);

    // gradients of shear strain
    grad.template block<3,3>(0, 0) = -inv_length * R.transpose();
    grad.template block<3,3>(0, 6) = inv_length * R.transpose();

    grad.template block<3,3>(0, 3) = RT_dp_ds_R * (exp_theta.transpose() + gam_theta * dtheta_dRi[0]);
    grad.template block<3,3>(0, 9) = RT_dp_ds_R * gam_theta * dtheta_dRi[1];


    /** Compute gradients of bending strain */

    // gradients of bending strain
    // gradient w.r.t. positions = 0
    grad.template block<3,3>(3, 0) = Mat3r::Zero();
    grad.template block<3,3>(3, 6) = Mat3r::Zero();

    // gradient w.r.t. orientation
    grad.template block<3,3>(3, 3) = dtheta_ds_dRi[0];
    grad.template block<3,3>(3, 9) = dtheta_ds_dRi[1];

    return grad;
}

template <int Order>
Vec3r RodElement<Order>::contactPoint(Real s_hat, const Vec3r& cp_local) const
{
    return position(s_hat) + orientation(s_hat) * cp_local;
}

template <int Order>
Vec3r RodElement<Order>::previousContactPoint(Real s_hat, const Vec3r& cp_local) const
{
    return previousPosition(s_hat) + previousOrientation(s_hat) * cp_local;
}

template <int Order>
Vec3r RodElement<Order>::contactPointVelocity(Real s_hat, const Vec3r& cp_local) const
{
    // linear velocity contribution
    Vec3r v = _bases[0](s_hat) * _nodes[0]->lin_velocity;
    for (int i = 1; i < Order+1; i++)
    {
        v += _bases[i](s_hat) * _nodes[i]->lin_velocity;
    }

    // stores (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Vec3r, Order+1> Ri_minus_R1;
    // stores Gamma^{-1} (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Mat3r, Order+1> gam_inv_Ri_minus_R1;
    // stores d theta(s) / d Ri for i = 1,...,Order+1
    std::array<Mat3r, Order+1> dtheta_dRi;

    // theta(s)
    Vec3r theta = Vec3r::Zero();

    // precompute quantities
    dtheta_dRi[0] = Mat3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        Ri_minus_R1[i] = Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
        gam_inv_Ri_minus_R1[i] = Math::ExpMap_InvRightJacobian(Ri_minus_R1[i]);
        dtheta_dRi[i] = _bases[i](s_hat) * gam_inv_Ri_minus_R1[i];

        theta += _bases[i](s_hat) * Ri_minus_R1[i];

        // add contribution to dtheta_dR0 and dtheta_ds_dR0
        dtheta_dRi[0] -= dtheta_dRi[i].transpose();
    }

    /** gradient of cp w.r.t. rotation * angular velocity */
    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R = _nodes[0]->orientation * exp_theta;

    for (int i = 0; i < Order+1; i++)
    {
        if (i == 0)
        {
            v += -R * Math::Skew3(cp_local) * (exp_theta.transpose() + gam_theta * dtheta_dRi[i]) * _nodes[i]->ang_velocity;
        }
        else
        {
            v += -R * Math::Skew3(cp_local) * gam_theta * dtheta_dRi[i] * _nodes[i]->ang_velocity;
        }
    }

    return v;
}

template <int Order>
Vec3r RodElement<Order>::previousContactPointVelocity(Real s_hat, const Vec3r& cp_local) const
{
    // linear velocity contribution
    Vec3r v = _bases[0](s_hat) * _nodes[0]->prev_lin_velocity;
    for (int i = 1; i < Order+1; i++)
    {
        v += _bases[i](s_hat) * _nodes[i]->prev_lin_velocity;
    }

    // stores (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Vec3r, Order+1> Ri_minus_R1;
    // stores Gamma^{-1} (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Mat3r, Order+1> gam_inv_Ri_minus_R1;
    // stores d theta(s) / d Ri for i = 1,...,Order+1
    std::array<Mat3r, Order+1> dtheta_dRi;

    // theta(s)
    Vec3r theta = Vec3r::Zero();

    // precompute quantities
    dtheta_dRi[0] = Mat3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        Ri_minus_R1[i] = Math::Minus_SO3(_nodes[i]->prev_orientation, _nodes[0]->prev_orientation);
        gam_inv_Ri_minus_R1[i] = Math::ExpMap_InvRightJacobian(Ri_minus_R1[i]);
        dtheta_dRi[i] = _bases[i](s_hat) * gam_inv_Ri_minus_R1[i];

        theta += _bases[i](s_hat) * Ri_minus_R1[i];

        // add contribution to dtheta_dR0 and dtheta_ds_dR0
        dtheta_dRi[0] -= dtheta_dRi[i].transpose();
    }

    /** gradient of cp w.r.t. rotation * angular velocity */
    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R = _nodes[0]->prev_orientation * exp_theta;

    for (int i = 0; i < Order+1; i++)
    {
        if (i == 0)
        {
            v += -R * Math::Skew3(cp_local) * (exp_theta.transpose() + gam_theta * dtheta_dRi[i]) * _nodes[i]->prev_ang_velocity;
        }
        else
        {
            v += -R * Math::Skew3(cp_local) * gam_theta * dtheta_dRi[i] * _nodes[i]->prev_ang_velocity;
        }
    }

    return v;
}

template <int Order>
typename RodElement<Order>::ContactPointGradientMatType RodElement<Order>::contactPointGradient(Real s_hat, const Vec3r& cp_local) const
{
    ContactPointGradientMatType grad;
    
    // stores (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Vec3r, Order+1> Ri_minus_R1;
    // stores Gamma^{-1} (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Mat3r, Order+1> gam_inv_Ri_minus_R1;
    // stores d theta(s) / d Ri for i = 1,...,Order+1
    std::array<Mat3r, Order+1> dtheta_dRi;

    // theta(s)
    Vec3r theta = Vec3r::Zero();

    // precompute quantities
    dtheta_dRi[0] = Mat3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        Ri_minus_R1[i] = Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
        gam_inv_Ri_minus_R1[i] = Math::ExpMap_InvRightJacobian(Ri_minus_R1[i]);
        dtheta_dRi[i] = _bases[i](s_hat) * gam_inv_Ri_minus_R1[i];

        theta += _bases[i](s_hat) * Ri_minus_R1[i];

        // add contribution to dtheta_dR0 and dtheta_ds_dR0
        dtheta_dRi[0] -= dtheta_dRi[i].transpose();
    }


    /** Compute gradients w.r.t. rotation */

    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R = _nodes[0]->orientation * exp_theta;

    for (int i = 0; i < Order+1; i++)
    {
        if (i == 0)
        {
            grad.template block<3,3>(0,6*i+3) = -R * Math::Skew3(cp_local) * (exp_theta.transpose() + gam_theta * dtheta_dRi[i]);
        }
        else
        {
            grad.template block<3,3>(0,6*i+3) = -R * Math::Skew3(cp_local) * gam_theta * dtheta_dRi[i];
        }
    }

    /** Compute gradients w.r.t. position */
    for (int i = 0; i < Order+1; i++)
    {
        grad.template block<3,3>(0,6*i) = _bases[i](s_hat) * Mat3r::Identity();
    }
    

    return grad;
}

template <int Order>
typename RodElement<Order>::ContactPointGradientMatType RodElement<Order>::contactPointVelocityGradient(Real s_hat, const Vec3r& cp_local) const
{
    // DOUBLE CHECK: gradient of contact point velocity w.r.t. velocities the same as contact point gradient?
    return contactPointGradient(s_hat, cp_local);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Specialization for "rigid body" rod element (Order = 0)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



template<>
Vec3r RodElement<0>::position(Real s_hat) const
{
    // the current length of the element factors in the stretch
    // stretch strain ==> 3rd component of [ 1/l * 1/2 * (R1 + R2)^T (p2 - p1) ]
    // therefore the current length of the element is 3rd component of 1/2 * (R1 + R2)^T (p2 - p1)
    Vec3r stretch = 0.5*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) * (_nodes[1]->position - _nodes[0]->position);
    Real current_length = stretch[2];

    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->position + s_hat * current_length * _nodes[0]->orientation.col(2);
    }
    else
    {
        return _nodes[1]->position - (1-s_hat) * current_length * _nodes[1]->orientation.col(2);
    }
}

template<>
Vec3r RodElement<0>::previousPosition(Real s_hat) const
{
    // the current length of the element factors in the stretch
    // stretch strain ==> 3rd component of [ 1/l * 1/2 * (R1 + R2)^T (p2 - p1) ]
    // therefore the current length of the element is 3rd component of 1/2 * (R1 + R2)^T (p2 - p1)
    Vec3r prev_stretch = 0.5*(_nodes[0]->prev_orientation.transpose() + _nodes[1]->prev_orientation.transpose()) * (_nodes[1]->prev_position - _nodes[0]->prev_position);
    Real prev_length = prev_stretch[2];

    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->prev_position + s_hat * prev_length * _nodes[0]->prev_orientation.col(2);
    }
    else
    {
        return _nodes[1]->prev_position - (1-s_hat) * prev_length * _nodes[1]->prev_orientation.col(2);
    }
}

template<>
Vec3r RodElement<0>::dposition_dshat(Real /* s_hat */) const
{
    // this doesn't make sense, throw an error
    throw std::runtime_error("dposition_dshat not defined for RodElement<0>");

    return Vec3r::Zero();
}

template<>
Mat3r RodElement<0>::orientation(Real s_hat) const
{
    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->orientation;
    }
    else
    {
        return _nodes[1]->orientation;
    }
}

template<>
Mat3r RodElement<0>::previousOrientation(Real s_hat) const
{
    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->prev_orientation;
    }
    else
    {
        return _nodes[1]->prev_orientation;
    }
}

template<>
Real RodElement<0>::Ni(int /* shape_func_index */, Real /* s_hat */) const
{
    // no shape functions for this type of element
    throw std::runtime_error("No shape functions defined for RodElement<0>");
}

template<>
Real RodElement<0>::dNi_dshat(int /* shape_func_index */, Real /* s_hat */ ) const
{
    // no shape functions for this type of element
    throw std::runtime_error("No shape functions defined for RodElement<0>");
}

template<>
Vec3r RodElement<0>::shearStrain(Real /* s_hat */) const
{
    Vec3r v = 0.5/_rest_length*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) * (_nodes[1]->position - _nodes[0]->position);
    return v - Vec3r(0,0,1);
}

template<>
Vec3r RodElement<0>::bendingStrain(Real /* s_hat */) const
{
    Vec3r u = 1.0/_rest_length * Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation); 
    return u;
}

template<>
Vec6r RodElement<0>::strain(Real s_hat) const
{
    Vec6r both;
    both.head<3>() = shearStrain(s_hat);
    both.tail<3>() = bendingStrain(s_hat);

    return both;
}

template<>
typename RodElement<0>::StrainGradientMatType RodElement<0>::strainGradient(Real /* s_hat */) const
{
    StrainGradientMatType grad;

    const Mat3r dCv_dp1 = -0.5*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) / _rest_length;
    const Mat3r dCv_dp2 = -dCv_dp1;
    
    const Vec3r dp = _nodes[1]->position - _nodes[0]->position;
    const Mat3r dCv_dor1 = 0.5*Math::Skew3(_nodes[0]->orientation.transpose() * dp / _rest_length);
    const Mat3r dCv_dor2 = 0.5*Math::Skew3(_nodes[1]->orientation.transpose() * dp / _rest_length);

    const Vec3r dtheta = Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation);
    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    const Mat3r dCu_dor1 = -jac_inv.transpose() / _rest_length;
    const Mat3r dCu_dor2 = jac_inv / _rest_length;

    // submatrices in correct spot in overall delC matrix
    grad.block<3,3>(0, 0) = dCv_dp1;
    grad.block<3,3>(0, 3) = dCv_dor1;
    grad.block<3,3>(0, 6) = dCv_dp2;
    grad.block<3,3>(0, 9) = dCv_dor2;
    grad.block<3,3>(3, 0) = Mat3r::Zero();
    grad.block<3,3>(3, 3) = dCu_dor1;
    grad.block<3,3>(3, 6) = Mat3r::Zero();
    grad.block<3,3>(3, 9) = dCu_dor2;

    return grad;
}

template<>
Vec3r RodElement<0>::contactPoint(Real s_hat, const Vec3r& cp_local) const
{
    return position(s_hat) + orientation(s_hat) * cp_local;
}

template<>
Vec3r RodElement<0>::previousContactPoint(Real s_hat, const Vec3r& cp_local) const
{
    return previousPosition(s_hat) + previousOrientation(s_hat) * cp_local;
}

template<>
Vec3r RodElement<0>::contactPointVelocity(Real s_hat, const Vec3r& cp_local) const
{
    Vec3r stretch = 0.5*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) * (_nodes[1]->position - _nodes[0]->position);
    Real l_c = stretch[2];

    // contact point = p1 + s_hat * l_c * R1 * e3 + R1 * cp_local

    // time derivative needs d/dt(l_c)  (i.e. the time derivative of the current element length)
    Vec3r ddt_v = 0.5 * ( _nodes[0]->orientation * Math::Skew3(_nodes[0]->ang_velocity) + _nodes[1]->orientation * Math::Skew3(_nodes[1]->ang_velocity)).transpose() * (_nodes[1]->position - _nodes[0]->position) + 
        (_nodes[0]->orientation + _nodes[1]->orientation).transpose() * (_nodes[1]->lin_velocity - _nodes[0]->lin_velocity);
    Real ddt_lc = ddt_v[2];

    if (s_hat < 0.5)
    {
        return _nodes[0]->lin_velocity + 
            s_hat * ddt_lc * _nodes[0]->orientation.col(2) + s_hat * l_c * _nodes[0]->orientation * Math::Skew3(_nodes[0]->ang_velocity).col(2) +
            _nodes[0]->orientation * Math::Skew3(_nodes[0]->ang_velocity) * cp_local;
    }
    else
    {
        return _nodes[1]->lin_velocity - 
            (1-s_hat) * ddt_lc * _nodes[1]->orientation.col(2) - s_hat * l_c * _nodes[1]->orientation * Math::Skew3(_nodes[1]->ang_velocity).col(2) +
            _nodes[1]->orientation * Math::Skew3(_nodes[1]->ang_velocity) * cp_local;
    }
}

template<>
typename RodElement<0>::ContactPointGradientMatType RodElement<0>::contactPointGradient(Real /* s_hat */, const Vec3r& /* cp_local */) const
{
    // this shouldn't be used
    throw std::runtime_error("contactPointGradient not defined for RodElement<0>");

    return ContactPointGradientMatType::Zero();
}

template class RodElement<0>;
template class RodElement<1>;
template class RodElement<2>;
template class RodElement<3>;

} // namespace SimObject