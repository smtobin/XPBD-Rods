#ifndef __MATH_HPP
#define __MATH_HPP

#include "common/common.hpp"


Mat3r Skew3(const Vec3r& vec)
{
    Mat3r mat;
    mat << 0,       -vec(2),    vec(1),
           vec(2),  0,          -vec(0),
           -vec(1), vec(0),     0;
    return mat;
}

Vec3r Vee3(const Mat3r& mat)
{
    return Vec3r(mat(2,1), mat(0,2), mat(1,0));
}

// Computes Jacobian of SO(3) exponential map
Mat3r ExpMap_Jacobian(const Vec3r& theta)
{
    Real theta_norm = theta.norm();
    const Mat3r skew = Skew3(theta);
    if (theta_norm < Real(1e-4))
    {
        return Mat3r::Identity() + 0.5 * skew;
    }
    
    return Mat3r::Identity() + (1 - std::cos(theta_norm)) / (theta_norm * theta_norm) * skew + (theta_norm - std::sin(theta_norm)) / (theta_norm * theta_norm * theta_norm) * skew * skew;
}

Mat3r Exp_so3(const Vec3r& vec)
{
    const Mat3r skew = Skew3(vec);
    Real mag = vec.norm();

    if (mag < Real(1e-6))
        return Mat3r::Identity() + skew;
    
    return Mat3r::Identity() + std::sin(mag) / mag * skew + (1 - std::cos(mag)) / (mag * mag) * skew * skew;
}

Vec3r Log_SO3(const Mat3r& mat)
{
    Real theta = std::acos(0.5 * mat.trace() - 0.5);

    if (std::abs(theta) < std::numeric_limits<Real>::epsilon())
    {
        return Vec3r::Zero();
    }

    const Mat3r skew_vec3 = Vee3(mat - mat.transpose());

    if (std::abs(mat.trace()) < Real(1e-4))
    {
        return 0.5 * (1 + theta*theta/6.0 + 7*theta*theta*theta*theta/360.0) * skew_vec3;
    }

    return theta / ( 2*std::sin(theta)) * skew_vec3;
}

Vec3r Minus_SO3(const Vec3r& mat1, const Vec3r& mat2)
{
    return Log_SO3(mat2.transpose() * mat1);
}

Mat3r Plus_SO3(const Mat3r& SO3_mat, const Vec3r& so3_vec)
{
    return SO3_mat * Exp_so3(so3_vec);
}

#endif // __MATH_HPP