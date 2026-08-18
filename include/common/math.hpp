#pragma once

#include "common/common.hpp"
#include <iostream>


class Math
{

public:

static Mat3r Skew3(const Vec3r& vec)
{
    Mat3r mat;
    mat << 0,       -vec(2),    vec(1),
           vec(2),  0,          -vec(0),
           -vec(1), vec(0),     0;
    return mat;
}

static Vec3r Vee3(const Mat3r& mat)
{
    return Vec3r(mat(2,1), mat(0,2), mat(1,0));
}

// Computes right Jacobian of SO(3) exponential map
static Mat3r ExpMap_RightJacobian(const Vec3r& theta)
{
    Real theta_norm = theta.norm();
    const Mat3r skew = Skew3(theta);
    if (theta_norm < Real(1e-8))
    {
        return Mat3r::Identity() - 0.5 * skew;
    }
    
    return Mat3r::Identity() - (1 - std::cos(theta_norm)) / (theta_norm * theta_norm) * skew + (theta_norm - std::sin(theta_norm)) / (theta_norm * theta_norm * theta_norm) * skew * skew;
}

// Computes derivative of right Jacobian of SO(3) exponential map (at theta), multiplied by some perturbation xi
static Mat3r DExpMap_RightJacobian_Contract_k(const Vec3r& theta, const Vec3r& xi)
{
    Real theta_norm = theta.norm();

    Mat3r skew_theta = Skew3(theta);
    Mat3r skew_xi = Skew3(xi);

    if (theta_norm < Real(1e-8))
    {
        return  1.0/12.0 * theta.dot(xi) * skew_theta - 0.5 * skew_xi;
    }

    Real theta_norm2 = theta_norm*theta_norm;
    Real theta_norm3 = theta_norm2*theta_norm;
    Real theta_norm4 = theta_norm3*theta_norm;

    Real a = (1 - std::cos(theta_norm)) / theta_norm2;
    Real b = (theta_norm - std::sin(theta_norm)) / theta_norm3;
    Real da_dtheta_norm = -2*(1 - std::cos(theta_norm)) / theta_norm3 + std::sin(theta_norm) / theta_norm2;
    Real db_dtheta_norm = (1 - std::cos(theta_norm)) / theta_norm3 - 3*(theta_norm - std::sin(theta_norm)) / theta_norm4;

    

    Mat3r term1 = -da_dtheta_norm * theta.dot(xi) / theta_norm * skew_theta;
    Mat3r term2 = -a * skew_xi;
    Mat3r term3 = db_dtheta_norm * theta.dot(xi) / theta_norm * skew_theta * skew_theta;
    Mat3r term4 = b * (skew_xi * skew_theta + skew_theta * skew_xi);

    return term1 + term2 + term3 + term4;
    
}

static Mat3r DExpMap_RightJacobian_Contract_j(const Vec3r& theta, const Vec3r& xi)
{
    Real theta_norm = theta.norm();

    if (theta_norm < Real(1e-2))
    {
        return DExpMap_RightJacobian_Contract_j_approx(theta, xi);
    }

    Mat3r skew_theta = Skew3(theta);

    Mat3r dskew_dtheta_i[3] = {Mat3r::Zero(), Mat3r::Zero(), Mat3r::Zero()};
    dskew_dtheta_i[0](1, 2) = -1;
    dskew_dtheta_i[0](2, 1) = 1;

    dskew_dtheta_i[1](0, 2) = 1;
    dskew_dtheta_i[1](2, 0) = -1;

    dskew_dtheta_i[2](0, 1) = -1;
    dskew_dtheta_i[2](1, 0) = 1;

    Real theta_norm2 = theta_norm*theta_norm;
    Real theta_norm3 = theta_norm2*theta_norm;
    Real theta_norm4 = theta_norm3*theta_norm;

    Real a = (1 - std::cos(theta_norm)) / theta_norm2;
    Real b = (theta_norm - std::sin(theta_norm)) / theta_norm3;
    Real da_dtheta_norm = -2*(1 - std::cos(theta_norm)) / theta_norm3 + std::sin(theta_norm) / theta_norm2;
    Real db_dtheta_norm = (1 - std::cos(theta_norm)) / theta_norm3 - 3*(theta_norm - std::sin(theta_norm)) / theta_norm4;


    Mat3r term1, term2, term3, term4;
    for (int k = 0; k < 3; k++)
    {
        term1.col(k) = -da_dtheta_norm * theta[k] / theta_norm * skew_theta * xi;
        term2.col(k) = -a * dskew_dtheta_i[k] * xi;
        term3.col(k) = db_dtheta_norm * theta[k] / theta_norm * skew_theta * skew_theta * xi;
        term4.col(k) = b * (dskew_dtheta_i[k] * skew_theta + skew_theta * dskew_dtheta_i[k]) * xi;
    }

    return term1 + term2 + term3 + term4;

}

static Mat3r DExpMap_RightJacobian_Contract_j_approx(const Vec3r& theta, const Vec3r& xi)
{
    Vec3r skew_xi = theta.cross(xi);

    Vec3r dskew_dtheta_xi_i[] = { Vec3r(0,-xi[2],xi[1]), Vec3r(xi[2],0,-xi[0]), Vec3r(-xi[1],xi[0],0) };
    Vec3r dskew_dtheta_skew_xi_i[] = { Vec3r(0,-skew_xi[2],skew_xi[1]), Vec3r(skew_xi[2],0,-skew_xi[0]), Vec3r(-skew_xi[1],skew_xi[0],0) };

    Mat3r term1, term2, term3;
    for (int k = 0; k < 3; k++)
    {
        term1.col(k) = 1.0/12.0 * theta[k] * skew_xi;
        term2.col(k) = -1.0/2.0 * dskew_dtheta_xi_i[k];
        term3.col(k) = 1.0/6.0 * (dskew_dtheta_skew_xi_i[k] + theta.cross(dskew_dtheta_xi_i[k]));
    }

    return term1 + term2 + term3;
}

// Computes the inverse of the right Jacobian of SO(3) exponential map
static Mat3r ExpMap_InvRightJacobian(const Vec3r& theta)
{
    Real theta_norm = theta.norm();
    const Mat3r skew = Skew3(theta);

    if (theta_norm < Real(1e-8))
    {
        return Mat3r::Identity() + 0.5 * skew;
    }

    return Mat3r::Identity() + 0.5*skew + (1/(theta_norm*theta_norm) - (1+std::cos(theta_norm)) / (2*theta_norm*std::sin(theta_norm)) ) * skew * skew;
}

static Mat3r Exp_so3(const Vec3r& vec)
{
    const Mat3r skew = Skew3(vec);
    Real mag = vec.norm();

    if (mag < Real(1e-8))
        return Mat3r::Identity() + skew;
    
    return Mat3r::Identity() + std::sin(mag) / mag * skew + (1 - std::cos(mag)) / (mag * mag) * skew * skew;
}

static Vec3r Log_SO3_Pi(const Mat3r& R)
{
    Vec3r u;

    if (R(0,0) >= R(1,1) && R(0,0) >= R(2,2))
    {
        Real x = std::sqrt(std::max(Real(0), (R(0,0)+1)/2));
        Real y = R(0,1) / (2*x);
        Real z = R(0,2) / (2*x);

        u << x, y, z;
    }
    else if (R(1,1) >= R(2,2))
    {
        Real y = std::sqrt(std::max(Real(0), (R(1,1)+1)/2));
        Real x = R(0,1) / (2*y);
        Real z = R(1,2) / (2*y);

        u << x, y, z;
    }
    else
    {
        Real z = std::sqrt(std::max(Real(0), (R(2,2)+1)/2));
        Real x = R(0,2) / (2*z);
        Real y = R(1,2) / (2*z);

        u << x, y, z;
    }

    u.normalize();

    return M_PI * u;
}

static Vec3r Log_SO3(const Mat3r& mat)
{
    // make sure 1/2 tr(mat) - 1/2 is not >1, will get NaNs. This may happen due to numerical drift
    Real c = std::clamp(
        0.5 * mat.trace() - 0.5,
        Real(-1),
        Real(1)
    );
    Real theta = std::acos(c);  

    const Vec3r skew_vec3 = Vee3(mat - mat.transpose());

    // handle singularity at theta=0
    if (std::abs(theta) < Real(1e-4))
    {
        return 0.5 * (1 + theta*theta/6.0 + 7*theta*theta*theta*theta/360.0) * skew_vec3;
    }

    // handle singularity at theta=pi (tr(R) = -1)
    if (std::abs(M_PI - theta) < Real(1e-4))
    {
        return Log_SO3_Pi(mat);
    }

    return theta / ( 2*std::sin(theta)) * skew_vec3;
}

static Vec3r Minus_SO3(const Mat3r& mat1, const Mat3r& mat2)
{
    return Log_SO3(mat2.transpose() * mat1);
}

static Mat3r Plus_SO3(const Mat3r& SO3_mat, const Vec3r& so3_vec)
{
    return SO3_mat * Exp_so3(so3_vec);
}

static Mat3r RotMatFromXYZEulerAngles(const Vec3r& euler_xyz)
{
    const Real x = euler_xyz(0) * M_PI / 180.0;
    const Real y = euler_xyz(1) * M_PI / 180.0;
    const Real z = euler_xyz(2) * M_PI / 180.0;

    // using the "123" convention: rotate first about x axis, then about y, then about z
    Mat3r rot_mat;
    rot_mat(0,0) = std::cos(y) * std::cos(z);
    rot_mat(0,1) = std::sin(x)*std::sin(y)*std::cos(z) - std::cos(x)*std::sin(z);
    rot_mat(0,2) = std::cos(x)*std::sin(y)*std::cos(z) + std::sin(x)*std::sin(z);

    rot_mat(1,0) = std::cos(y)*std::sin(z);
    rot_mat(1,1) = std::sin(x)*std::sin(y)*std::sin(z) + std::cos(x)*std::cos(z);
    rot_mat(1,2) = std::cos(x)*std::sin(y)*std::sin(z) - std::sin(x)*std::cos(z);

    rot_mat(2,0) = -std::sin(y);
    rot_mat(2,1) = std::sin(x)*std::cos(y);
    rot_mat(2,2) = std::cos(x)*std::cos(y);

    return rot_mat;
}

static Vec3r XYZEulerAnglesFromRotMat(const Mat3r& R)
{
    Real theta_y = std::asin(-R(2,0));
    Real theta_x, theta_z;
    if (std::abs(R(2,0)) < 1)
    {
        theta_x = std::atan2(R(2,1), R(2,2));
        theta_z = std::atan2(R(1,0), R(0,0));
    }
    else
    {
        theta_x = 0;
        if (R(2,0) == -1)
        {
            theta_y = M_PI/2;
            theta_z = std::atan2(-R(0,1), R(0,2));
        }
        else
        {
            theta_y = -M_PI/2;
            theta_z = std::atan2(R(0,1), -R(0,2));
        }
    }

    return 180/M_PI * Vec3r(theta_x, theta_y, theta_z);
}

/** Projects a point p onto the line segment defined by ab.
 * Returns the interpolation factor - i.e. if in [0,1] the projected point is between a and b.
 * To get the projected point: p_proj = a + projectPointOnLine(p, a, b) * (b-a);
 */
static Real projectPointOntoLine(const Vec3r& p, const Vec3r& a, const Vec3r& b)
{
    return (p-a).dot(b-a) / (b-a).squaredNorm();
}

/** Find closest points on two line segments defined by (p1, p2) and (p3, p4) */
static std::pair<Real, Real> findClosestPointsOnLineSegments(const Vec3r& p1, const Vec3r& p2, const Vec3r& p3, const Vec3r& p4)
{
    const Vec3r d1 = p2 - p1;
    const Vec3r d2 = p4 - p3;
    const Vec3r w = p1 - p3;

    Real a = d1.dot(d1);
    Real b = d1.dot(d2);
    Real c = d2.dot(d2);
    Real d = w.dot(d1);
    Real e = w.dot(d2);
    
    Real den = a*c - b*b;

    Real beta1, beta2;
    if (den < 1e-8) // line segments are roughly parallel
    {
        Real best_dist_sq = std::numeric_limits<Real>::max();

        // project p1 onto (p3,p4)
        Real beta2_1 = std::clamp(projectPointOntoLine(p1, p3, p4), 0.0, 1.0);
        Real dist_sq1 = ((1-beta2_1)*p3 + beta2_1*p4 - p1).squaredNorm();
        best_dist_sq = dist_sq1;
        beta1 = 0.0;
        beta2 = beta2_1;

        // project p2 onto (p3, p4)
        Real beta2_2 = std::clamp(projectPointOntoLine(p2, p3, p4), 0.0, 1.0);
        Real dist_sq2 = ((1-beta2_2)*p3 + beta2_2*p4 - p2).squaredNorm();
        if (dist_sq2 < best_dist_sq)
        {
            best_dist_sq = dist_sq2;
            beta1 = 1.0;
            beta2 = beta2_2;
        }

        // project p3 onto (p1, p2)
        Real beta1_1 = std::clamp(projectPointOntoLine(p3, p1, p2), 0.0, 1.0);
        Real dist_sq3 = ((1-beta1_1)*p1 + beta1_1*p2 - p3).squaredNorm();
        if (dist_sq3 < best_dist_sq)
        {
            best_dist_sq = dist_sq3;
            beta1 = beta1_1;
            beta2 = 0.0;
        }

        // project p4 onto (p1, p2)
        Real beta1_2 = std::clamp(projectPointOntoLine(p4, p1, p2), 0.0, 1.0);
        Real dist_sq4 = ((1-beta1_2)*p1 + beta1_2*p2 - p4).squaredNorm();
        if (dist_sq4 < best_dist_sq)
        {
            best_dist_sq = dist_sq4;
            beta1 = beta1_2;
            beta2 = 1.0;
        }
    }
    else
    {
        beta1 = (b*e - c*d) / den;
        beta2 = (a*e - b*d) / den;

        if (beta1 < 0 || beta1 > 1)
        {
            // if beta1 was clamped, re-project to find beta2
            beta1 = std::clamp(beta1, 0.0, 1.0);
            beta2 = std::clamp(projectPointOntoLine((1-beta1)*p1 + beta1*p2, p3, p4), 0.0, 1.0);
            
        }
        else if (beta2 < 0 || beta2 > 1)
        {
            // if beta2 was clamped, re-project to find beta1
            beta2 = std::clamp(beta2, 0.0, 1.0);
            beta1 = std::clamp(projectPointOntoLine((1-beta2)*p3 + beta2*p4, p1, p2), 0.0, 1.0);
        }
    }

    return std::make_pair(beta1, beta2);
}

static Vec4r rotationMatrixToQuaternion(const Mat3r& mat)
{
    Vec4r q;

    // code adapted from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    const Real trace = mat.trace();
    if( trace > 0 )
    {
        Real s = Real(0.5) / std::sqrt(trace + 1);
        q[3] = Real(0.25) / s;
        q[0] = ( mat(2,1) - mat(1,2) ) * s;
        q[1] = ( mat(0,2) - mat(2,0) ) * s;
        q[2] = ( mat(1,0) - mat(0,1) ) * s;
    } 
    else
    {
        if ( mat(0,0) > mat(1,1) && mat(0,0) > mat(2,2) ) {
            Real s = 2 * std::sqrt( 1 + mat(0,0) - mat(1,1) - mat(2,2));
            q[3] = (mat(2,1) - mat(1,2) ) / s;
            q[0] = Real(0.25) * s;
            q[1] = (mat(0,1) + mat(1,0) ) / s;
            q[2] = (mat(0,2) + mat(2,0) ) / s;
        } else if (mat(1,1) > mat(2,2)) {
            Real s = 2 * std::sqrt( 1 + mat(1,1) - mat(0,0) - mat(2,2));
            q[3] = (mat(0,2) - mat(2,0) ) / s;
            q[0] = (mat(0,1) + mat(1,0) ) / s;
            q[1] = 0.25f * s;
            q[2] = (mat(1,2) + mat(2,1) ) / s;
        } else {
            Real s = 2 * std::sqrt( 1 + mat(2,2) - mat(0,0) - mat(1,1) );
            q[3] = (mat(1,0) - mat(0,1) ) / s;
            q[0] = (mat(0,2) + mat(2,0) ) / s;
            q[1] = (mat(1,2) + mat(2,1) ) / s;
            q[2] = 0.25f * s;
        }
    }
    
    return q;
}

static Mat3r quaternionToRotationMatrix(const Vec4r& quat)
{
    Mat3r mat;
    mat(0,0) = 1 - 2*quat[1]*quat[1] - 2*quat[2]*quat[2];
    mat(0,1) = 2*quat[0]*quat[1] - 2*quat[3]*quat[2];
    mat(0,2) = 2*quat[0]*quat[2] + 2*quat[3]*quat[1];
    mat(1,0) = 2*quat[0]*quat[1] + 2*quat[3]*quat[2];
    mat(1,1) = 1 - 2*quat[0]*quat[0] - 2*quat[2]*quat[2];
    mat(1,2) = 2*quat[1]*quat[2] - 2*quat[3]*quat[0];
    mat(2,0) = 2*quat[0]*quat[2] - 2*quat[3]*quat[1];
    mat(2,1) = 2*quat[1]*quat[2] + 2*quat[3]*quat[0];
    mat(2,2) = 1 - 2*quat[0]*quat[0] - 2*quat[1]*quat[1];

    return mat;
}

// =========== GEOMETRY UTILS ==============

static Vec3r pointSegmentDirection(const Vec3r &x0, const Vec3r &x1, const Vec3r &x2)
{
    const Vec3r &dx(x2 - x1);
    Real m2 = dx.squaredNorm();
    // find parameter value of closest point on segment
    Real s12 = (Real)dx.dot(x2 - x0) / m2;
    // cap parameter value to [0,1]
    if (s12 < 0)
        s12 = 0;
    else if (s12 > 1)
        s12 = 1;

    // and find the distance
    const Vec3r closest_point_on_line = s12 * x1 + (1 - s12) * x2;
    return (x0 - closest_point_on_line).normalized();
}

static Real pointSegmentDistance(const Vec3r &x0, const Vec3r &x1, const Vec3r &x2)
{
    const Vec3r dx = x2 - x1;
    Real m2 = dx.squaredNorm();
    // find parameter value of closest point on segment
    Real s12 = (Real)dx.dot(x2 - x0) / m2;
    // cap parameter value to [0,1]
    if (s12 < 0)
        s12 = 0;
    else if (s12 > 1)
        s12 = 1;

    // and find the distance
    const Vec3r closest_point_on_line = s12 * x1 + (1 - s12) * x2;
    return (x0 - closest_point_on_line).norm();
}

/** Returns the minimum distance from a point to a triangle.
 * @param x0 - the point
 * @param x1 - 1st triangle vertex
 * @param x2 - 2nd triangle vertex
 * @param x3 - 3rd triangle vertex
 */
static Vec3r pointTriangleDirection(const Vec3r &x0, const Vec3r &x1, const Vec3r &x2, const Vec3r &x3)
{
    // first find barycentric coordinates of closest point on infinite plane
    const Vec3r x13(x1 - x3), x23(x2 - x3), x03(x0 - x3);
    Real m13 = x13.squaredNorm(), m23 = x23.squaredNorm(), d = x13.dot(x23);
    Real invdet = 1.f / std::max(m13 * m23 - d * d, Real(1e-30));
    Real a = x13.dot(x03), b = x23.dot(x03);

    // the barycentric coordinates themselves
    Real w23 = invdet * (m23 * a - d * b);
    Real w31 = invdet * (m13 * b - d * a);
    Real w12 = 1 - w23 - w31;

    if (w23 >= 0 && w31 >= 0 && w12 >= 0) // if we're inside the triangle
    {
        const Vec3r closest_point_on_triangle = w23 * x1 + w31 * x2 + w12 * x3;
        const Vec3r diff = x0 - closest_point_on_triangle;
        const Real dist = diff.norm();
        if (dist < GEOMETRY_EPS) // check if we're on the plane - if so use the plane normal as the gradient
        {
            const Vec3r x21 = x2 - x1;
            const Vec3r x31 = x3 - x1;
            const Vec3r n = x21.cross(x31).normalized();
            return n;
        }
        return diff / dist;
    }
    else // we have to clamp to one of the edges
    {
        if (w23 > 0) // this rules out edge 2-3 for us
            if (pointSegmentDistance(x0, x1, x2) < pointSegmentDistance(x0, x1, x3))
                return pointSegmentDirection(x0, x1, x2);
            else
                return pointSegmentDirection(x0, x1, x3);
        else if (w31 > 0) // this rules out edge 1-3
            if (pointSegmentDistance(x0,x1,x2) < pointSegmentDistance(x0,x2,x3))
                return pointSegmentDirection(x0,x1,x2);
            else
                return pointSegmentDirection(x0,x2,x3);
        else // w12 must be >0, ruling out edge 1-2
            if (pointSegmentDistance(x0,x1,x3) < pointSegmentDistance(x0,x2,x3))
                return pointSegmentDirection(x0,x1,x3);
            else
                return pointSegmentDirection(x0,x2,x3);
    }
}

static Real pointTriangleDistance(const Vec3r &x0, const Vec3r &x1, const Vec3r &x2, const Vec3r &x3)
{
    // first find barycentric coordinates of closest point on infinite plane
    const Vec3r x13(x1 - x3), x23(x2 - x3), x03(x0 - x3);
    Real m13 = x13.squaredNorm(), m23 = x23.squaredNorm(), d = x13.dot(x23);
    Real invdet = 1.f / std::max(m13 * m23 - d * d, Real(1e-30));
    Real a = x13.dot(x03), b = x23.dot(x03);

    // the barycentric coordinates themselves
    Real w23 = invdet * (m23 * a - d * b);
    Real w31 = invdet * (m13 * b - d * a);
    Real w12 = 1 - w23 - w31;

    if (w23 >= 0 && w31 >= 0 && w12 >= 0) // if we're inside the triangle
    {
        const Vec3r closest_point_on_triangle = w23 * x1 + w31 * x2 + w12 * x3;
        return (x0 - closest_point_on_triangle).norm();
    }
    else // we have to clamp to one of the edges
    {
        if (w23 > 0) // this rules out edge 2-3 for us
            return std::min(pointSegmentDistance(x0, x1, x2), pointSegmentDistance(x0, x1, x3));
        else if (w31 > 0) // this rules out edge 1-3
            return std::min(pointSegmentDistance(x0, x1, x2), pointSegmentDistance(x0, x2, x3));
        else // w12 must be >0, ruling out edge 1-2
            return std::min(pointSegmentDistance(x0, x1, x3), pointSegmentDistance(x0, x2, x3));
    }
}

/** Returns true if point (x0,y0) is in the triangle defined by (x1,y1), (x2,y2), (x3,y3).
 * The barycentric coordinates (a,b,c) are computed.
 * @param x0,y0 - the point
 * @param x1,y1 - the 1st triangle vertex
 * @param x2,y2 - the 2nd triangle vertex
 * @param x3,y3 - the 3rd triangle vertex
 * @param a,b,c (OUTPUT) - the barycentric coordinates of (x0,y0) in the triangle.
 */
static bool pointInTriangle2D(Real x0, Real y0,
                       Real x1, Real y1, Real x2, Real y2, Real x3, Real y3,
                       Real &a, Real &b, Real &c)
{
    a = ((y2 - y3) * (x0 - x3) + (x3 - x2) * (y0 - y3)) / ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));
    b = ((y3 - y1) * (x0 - x3) + (x1 - x3) * (y0 - y3)) / ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));
    c = 1 - a - b;

    return (a + GEOMETRY_EPS) >= 0 && a <= 1 && (b + GEOMETRY_EPS) >= 0 && b <= 1 && (c + GEOMETRY_EPS) >= 0 && c <= 1;
}


static Real angleBetweenVectors(const Vec3r& v1, const Vec3r& v2)
{
    const Real dot = v1.dot(v2);
    const Real det = v1.cross(v2).norm();
    return std::atan2(det, dot);
}

static Vec3r vectorSlerp(const Vec3r& v1, const Vec3r& v2, Real t)
{
    const Real angle = angleBetweenVectors(v1, v2);
    // in the unlikely case that v1 and v2 are colinear (i.e. have angle between them of 180 deg)
    if (M_PI - std::abs(angle) < 1e-6)
    {
        // from https://math.stackexchange.com/a/211195 - find perpendicular vector
        Vec3r perp_v(v1[2], v1[2], -v1[0]-v1[1]);
        if (perp_v.squaredNorm() < GEOMETRY_EPS) perp_v = Vec3r(-v1[1]-v1[2], v1[0], v1[0]);
        perp_v.normalize();

        // this perpendicular vector is the halfway point
        if (t < 0.5f)   return vectorSlerp(v1, perp_v, t*2.0);
        else            return vectorSlerp(perp_v, v2, (t-0.5)*2.0);
    }
    // if angle is 0 between them, just return v1
    else if (std::abs(angle) < 1e-6)
    {
        return v1;
    }

    const Real inv_sin_ang = 1 / std::sin(angle);
    return std::sin( (1-t) * angle) * inv_sin_ang * v1 + std::sin(t*angle) * inv_sin_ang * v2;
}

static Vec3r vectorBiSlerp(const Vec3r& v00, const Vec3r& v10,
                              const Vec3r& v01, const Vec3r& v11,
                              Real t0, Real t1)
{
    return vectorSlerp( vectorSlerp(v00, v10, t0),
                        vectorSlerp(v01, v11, t0),
                        t1   );
}

static Vec3r vectorTriSlerp( const Vec3r& v000, const Vec3r& v100,
                                const Vec3r& v010, const Vec3r& v110,
                                const Vec3r& v001, const Vec3r& v101,
                                const Vec3r& v011, const Vec3r& v111,
                                Real t0, Real t1, Real t2)
{
    return vectorSlerp( vectorBiSlerp(v000, v100, v010, v110, t0, t1),
                        vectorBiSlerp(v001, v101, v011, v111, t0, t1),
                        t2  );
}
};