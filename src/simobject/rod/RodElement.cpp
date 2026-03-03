#include "simobject/rod/RodElement.hpp"

namespace SimObject
{

Real linearN1(Real s_hat) { return -s_hat + 1; }
Real linearN2(Real s_hat) { return s_hat; }
Real dlinearN1(Real s_hat) { return -1; }
Real dlinearN2(Real s_hat) { return 1; }

Real quadraticN1(Real s_hat) { return 2*s_hat*s_hat - 3*s_hat + 1; }
Real quadraticN2(Real s_hat) { return -4*s_hat*s_hat + 4*s_hat; }
Real quadraticN3(Real s_hat) { return 2*s_hat*s_hat - s_hat; }
Real dquadraticN1(Real s_hat) { return 4*s_hat - 3; }
Real dquadraticN2(Real s_hat) { return -8*s_hat + 4; }
Real dquadraticN3(Real s_hat) { return 4*s_hat - 1; }

template<int Order>
RodElement<Order>::RodElement(std::initializer_list<const SimObject::OrientedParticle*> nodes_list, Real rest_length)
    : _nodes{}, _bases{}, _bases_derivatives{}, _rest_length(rest_length)
{
    std::copy(nodes_list.begin(), nodes_list.end(), _nodes.begin());

    // get pointers to basis functions and their derivatives, according to element order
    if constexpr (Order == 1)
    {
        _bases = {linearN1, linearN2};
        _bases_derivatives = {dlinearN1, dlinearN2};
    }
    else if constexpr (Order == 2)
    {
        _bases = {quadraticN1, quadraticN2, quadraticN3};
        _bases_derivatives = {dquadraticN1, dquadraticN2, dquadraticN3};
    }
    else
    {
        static_assert(0);
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

template <int Order>
Mat3r RodElement<Order>::orientation(Real s_hat) const
{
    // get interpolated relative rotation vector
    Vec3r theta = Vec3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        theta += _bases[i](s_hat) * Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
    }

    // use exponential map to get interpolated rotation
    return Math::Plus_SO3(_nodes[0]->orientation, theta);
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

    return R.transpose() * dshat_ds() * dp_ds;
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

    return Math::ExpMap_RightJacobian(theta) * dtheta_ds;
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
        dtheta_ds_dRi[0] -= -dtheta_ds_dRi[i].transpose();
    }


    /** Compute gradients of shear strain */

    // precompute useful quantities for gradients of shear strain
    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R0_gam_theta = _nodes[0]->orientation * gam_theta;
    Mat3r R = _nodes[0]->orientation * exp_theta;


    Vec3r dp_ds = Vec3r::Zero();
    for (int i = 0; i < Order+1; i++)
    {
        dp_ds += _bases_derivatives[i](s_hat) * _nodes[i]->position;
    }
    Mat3r RT_dp_ds_R = R.transpose() * Math::Skew3(dp_ds) * R;

    // gradients of shear strain
    for (int i = 0; i < Order+1; i++)
    {
        grad.template block<3,3>(0,6*i) = R.transpose() * dshat_ds() * _bases_derivatives[i](s_hat);

        if (i == 0)
        {
            grad.template block<3,3>(0,6*i+3) = RT_dp_ds_R * (exp_theta + R0_gam_theta * dtheta_dRi[i]);
        }
        else
        {
            grad.template block<3,3>(0,6*i+3) = RT_dp_ds_R * R0_gam_theta * dtheta_dRi[i];
        }
    }


    /** Compute gradients of bending strain */

    // gradients of bending strain
    for (int i = 0; i < Order+1; i++)
    {
        // gradient w.r.t. positions = 0
        grad.template block<3,3>(3, 6*i) = Mat3r::Zero();

        // gradient w.r.t. orientation
        Vec3r xi = dtheta_dRi[i] * dtheta_ds;
        grad.template block<3,3>(3, 6*i+3) = Math::DExpMap_RightJacobian(theta, xi) + gam_theta * dtheta_ds_dRi[i];
    }

    return grad;
}

template class RodElement<1>;
template class RodElement<2>;

} // namespace SimObject