/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Jia Pan */

/** Implementation of box-box collisions inspired (copied) from FCL collision library: 
 * https://github.com/flexible-collision-library/fcl/blob/master/include/fcl/narrowphase/detail/primitive_shape_algorithm/box_box-inl.h
 */

#pragma once

#include "collision/CollisionTypes.hpp"

namespace Collision
{

class BoxBoxCollider
{

public:
    static std::vector<DetectedCollision> collideBoxes(
        SimObject::OrientedParticle* box1_com, const Vec3r& sbox1,
        SimObject::OrientedParticle* box2_com, const Vec3r& sbox2
    )
    {
        // extract position and orientation from COM
        const Vec3r& pbox1 = box1_com->position;
        const Vec3r& pbox2 = box2_com->position;

        const Mat3r& Rbox1 = box1_com->orientation;
        const Mat3r& Rbox2 = box2_com->orientation;

        // empty vector of collisions
        std::vector<DetectedCollision> collisions;

        // compute half-sizes of boxes
        Vec3r hsbox1 = sbox1 / 2.0;
        Vec3r hsbox2 = sbox2 / 2.0;

        // separating axis theorem - look for axes where projections don't overlap
        // 15 potential separating axes:
        //  - 3 face normals of box 1
        //  - 3 face normals of box 2
        //  - 9 edge cross products (3 edges of box 1 x 3 edges of box 2)
        
        Real min_penetration = std::numeric_limits<Real>::max();
        Vec3r normal = Rbox1.col(0);
        Vec3r center_diff = pbox2 - pbox1;
        bool flip_normal = false;
        int code;

        auto test_axis = [&](const Vec3r& axis, int cur_code) -> bool
        {
            Real proj1 =    hsbox1[0] * std::abs(Rbox1.col(0).dot(axis)) +
                            hsbox1[1] * std::abs(Rbox1.col(1).dot(axis)) +
                            hsbox1[2] * std::abs(Rbox1.col(2).dot(axis));
            Real proj2 =    hsbox2[0] * std::abs(Rbox2.col(0).dot(axis)) +
                            hsbox2[1] * std::abs(Rbox2.col(1).dot(axis)) +
                            hsbox2[2] * std::abs(Rbox2.col(2).dot(axis));
            Real dist = center_diff.dot(axis);
            Real dist_abs = std::abs(dist);

            Real overlap = proj1 + proj2 - dist_abs;

            if (overlap < 0)
                return false;
            
            if (overlap < min_penetration)
            {
                min_penetration = overlap;
                normal = axis;
                code = cur_code;
                flip_normal = dist < 0;
            }

            return true;
        };

        // 3 face normals of box 1 and 2
        for (int i = 0; i < 3; i++)
        {
            if (!test_axis(Rbox1.col(i), i+1))
                return collisions;

            if (!test_axis(Rbox2.col(i), i+4))
                return collisions;
        }

        // edge cross products
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Vec3r axis = Rbox1.col(i).cross(Rbox2.col(j));
                Real sq_len = axis.squaredNorm();
                // skip near-parallel edges
                if (sq_len < 1e-6)
                    continue;

                axis = axis / std::sqrt(sq_len);

                if (!test_axis(axis, 7 + 3*i+j))
                    return collisions;
            }
        }

        // all axes overlap ==> collision!

        // always want the normal pointing from box1 to box2
        // if the minimum separating axis dotted with the vector (box2_pos - box1_pos) was negative, we need to flip the normal
        if (flip_normal)
            normal = -normal;
        
        /** Compute contact point(s) */

        // Edge-edge contact!
        if (code > 6)
        {
            // find point p1 on the intersecting edge of box 1
            Vec3r p1(pbox1);
            for (int k = 0; k < 3; k++)
            {
                int sign = (Rbox1.col(k).dot(normal) > 0) ? 1 : -1;
                p1 += Rbox1.col(k) * (hsbox1[k] * sign);
            }

            // find point p2 on the intersecting edge of box 2
            Vec3r p2(pbox2);
            for (int k = 0; k < 3; k++)
            {
                int sign = (Rbox2.col(k).dot(normal) > 0) ? -1 : 1;
                p2 += Rbox2.col(k) * (hsbox2[k] * sign);
            }

            // directions of intersecting edges for each box ==> just the columns of the rotation matrix
            Vec3r u1(Rbox1.col((code - 7)/3));
            Vec3r u2(Rbox2.col((code - 7)%3));

            // find the closest point between the lines defined by p1 in the direction of u1, and p2 in the direction of u2
            // returns two interpolation parameters that will give us the contact points
            const auto [alpha, beta] = _closestPointBetweenLines(p1, u1, p2, u2);

            // p1 and p2 become the contact points (move along each respective edge to get to the contact point)
            p1 += alpha * u1;
            p2 += beta * u2;

            Vec3r p1_local = Rbox1.transpose() * (p1 - pbox1);
            Vec3r p2_local = Rbox2.transpose() * (p2 - pbox2);

            RigidRigidCollision new_collision;
            new_collision.cp_local1 = p1_local;
            new_collision.cp_local2 = p2_local;
            new_collision.normal = normal;
            new_collision.particle1 = box1_com;
            new_collision.particle2 = box2_com;
            collisions.push_back(std::move(new_collision));

            return collisions;
        }


        // Face-something contact (non-edge-edge contact)
        Mat3r R1, R2;
        Vec3r p1, p2;
        Vec3r halfsize1, halfsize2;

        SimObject::OrientedParticle* com1;
        SimObject::OrientedParticle* com2;

        // normal corresponds to a face on box 1
        if (code <= 3)
        {
            R1 = Rbox1;
            R2 = Rbox2;
            p1 = pbox1;
            p2 = pbox2;
            halfsize1 = hsbox1;
            halfsize2 = hsbox2;

            com1 = box1_com;
            com2 = box2_com;
        }
        // normal corresponds to a face on box 2
        else
        {
            R1 = Rbox2;
            R2 = Rbox1;
            p1 = pbox2;
            p2 = pbox1;
            halfsize1 = hsbox2;
            halfsize2 = hsbox1;

            com1 = box2_com;
            com2 = box1_com;
        }

        // normal vector of the reference face
        Vec3r normal2 = normal;
        if (code >= 4)
            // needs to be flipped if the reference face is on box 2
            normal2 = -normal;

        // normal vector of reference face dotted with axes of incident box
        Vec3r nr = R2.transpose() * normal2;
        // absolute value of "   "
        Vec3r anr = nr.cwiseAbs();

        // find the largest component of anr: this corresponds to the normal for the incident face
        // the other axis numbers of the incident face are stored in a1, a2
        int lanr, a1, a2;
        if(anr[1] > anr[0])
        {
            if(anr[1] > anr[2])
            {
                a1 = 0;
                lanr = 1;
                a2 = 2;
            }
            else
            {
                a1 = 0;
                a2 = 1;
                lanr = 2;
            }
        }
        else
        {
            if(anr[0] > anr[2])
            {
                lanr = 0;
                a1 = 1;
                a2 = 2;
            }
            else
            {
                a1 = 0;
                a2 = 1;
                lanr = 2;
            }
        }

        // compute the center point of incident face, in reference-face coordinates
        Vec3r center;
        if (nr[lanr] < 0)
            center = p2 - p1 + R2.col(lanr) * (halfsize2[lanr]);
        else
            center = p2 - p1 - R2.col(lanr) * (halfsize2[lanr]);

        // find the normal and non-normal axis numbers of the reference box
        int codeN, code1, code2;
        if(code <= 3)
            codeN = code-1;
        else 
            codeN = code-4;

        if(codeN == 0)
        {
            code1 = 1;
            code2 = 2;
        }
        else if(codeN == 1)
        {
            code1 = 0;
            code2 = 2;
        }
        else
        {
            code1 = 0;
            code2 = 1;
        }

        // find the four corners of the incident face, in reference-face coordinates
        Real quad[8]; // 2D coordinate of incident face (x,y pairs)
        Real c1, c2, m11, m12, m21, m22;
        c1 = R1.col(code1).dot(center);
        c2 = R1.col(code2).dot(center);
        // optimize this? - we have already computed this data above, but it is not
        // stored in an easy-to-index format. for now it's quicker just to recompute
        // the four dot products.
        Vec3r tempRac = R1.col(code1);
        m11 = R2.col(a1).dot(tempRac);
        m12 = R2.col(a2).dot(tempRac);
        tempRac = R1.col(code2);
        m21 = R2.col(a1).dot(tempRac);
        m22 = R2.col(a2).dot(tempRac);

        Real k1 = m11 * halfsize2[a1];
        Real k2 = m21 * halfsize2[a1];
        Real k3 = m12 * halfsize2[a2];
        Real k4 = m22 * halfsize2[a2];
        quad[0] = c1 - k1 - k3;
        quad[1] = c2 - k2 - k4;
        quad[2] = c1 - k1 + k3;
        quad[3] = c2 - k2 + k4;
        quad[4] = c1 + k1 + k3;
        quad[5] = c2 + k2 + k4;
        quad[6] = c1 + k1 - k3;
        quad[7] = c2 + k2 - k4;

        // find the size of the reference face
        Real rect[2];
        rect[0] = halfsize1[code1];
        rect[1] = halfsize1[code2];

        // intersect the incident and reference faces
        Real ret[16];
        int n_intersect = _intersectRectQuad2(rect, quad, ret);
        if(n_intersect < 1) { std::cerr << "n_intersect < 1 assert(0);"<< std::endl; assert(0); return collisions; } // this should never happen

        // convert the intersection points into reference-face coordinates,
        // and compute the contact position and depth for each point. only keep
        // those points that have a positive (penetrating) depth. delete points in
        // the 'ret' array as necessary so that 'point' and 'ret' correspond.
        Vec3r points[8]; // penetrating contact points
        Real dep[8]; // depths for those points
        Real det1 = 1.f/(m11*m22 - m12*m21);
        m11 *= det1;
        m12 *= det1;
        m21 *= det1;
        m22 *= det1;
        int cnum = 0;	// number of penetrating contact points found
        for(int j = 0; j < n_intersect; ++j)
        {
            Real k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
            Real k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
            points[cnum] = center + R2.col(a1) * k1 + R2.col(a2) * k2;
            dep[cnum] = halfsize1[codeN] - normal2.dot(points[cnum]);
            if(dep[cnum] >= 0)
            {
                ret[cnum*2] = ret[j*2];
                ret[cnum*2+1] = ret[j*2+1];
                cnum++;
            }
        }
        if(cnum < 1) { std::cerr << "cnum < 1 assert(0);"<< std::endl; assert(0); return collisions; } // this should never happen

        // we can't generate more contacts than we actually have
        int maxc = 4;
        if(maxc > cnum) maxc = cnum;
        if(maxc < 1) maxc = 1;

        // The determination of these contact computations are tested in:
        // test_fcl_box_box.cpp.
        //  The case where cnum <= maxc is tested by the test:
        //    test_collision_box_box_all_contacts
        //  The case where cnum > maxc is tested by the test:
        //    test_collision_box_box_cull_contacts
        //
        // Each of those tests is exercised twice: once when code < 4 and once when
        // 4 <= code < 7.
        int iret[] = {0, 1, 2, 3, 4, 5, 6, 7};
        if (cnum > maxc)
        {
            int i1 = 0;
            Real maxdepth = dep[0];
            for(int i = 1; i < cnum; ++i)
            {
                if(dep[i] > maxdepth)
                {
                    maxdepth = dep[i];
                    i1 = i;
                }
            }

            _cullPoints2(cnum, ret, maxc, i1, iret);
            cnum = maxc;
        }

        if (code < 4)
        {
            for(int j = 0; j < cnum; ++j)
            {
                int i = iret[j];
                Vec3r cp2_global = points[i] + p1;
                Vec3r cp2_local = R2.transpose() * (cp2_global - p2);
                Vec3r cp1_global = cp2_global + normal * dep[i];
                Vec3r cp1_local = R1.transpose() * (cp1_global - p1);

                RigidRigidCollision new_collision;
                new_collision.cp_local1 = cp1_local;
                new_collision.cp_local2 = cp2_local;
                new_collision.normal = normal;
                new_collision.particle1 = com1;
                new_collision.particle2 = com2;
                collisions.push_back(std::move(new_collision));
            }
        } 
        else 
        {
            for(int j = 0; j < cnum; ++j)
            {
                int i = iret[j];
                Vec3r cp2_global = points[i] + p1;
                Vec3r cp2_local = R2.transpose() * (cp2_global - p2);
                Vec3r cp1_global = cp2_global - normal * dep[i];
                Vec3r cp1_local = R1.transpose() * (cp1_global - p1);

                RigidRigidCollision new_collision;
                new_collision.cp_local1 = cp1_local;
                new_collision.cp_local2 = cp2_local;
                new_collision.normal = -normal;
                new_collision.particle1 = com1;
                new_collision.particle2 = com2;
                collisions.push_back(std::move(new_collision));
            }
        }

        return collisions;
    }

    static std::pair<Real, Real> _closestPointBetweenLines(const Vec3r& p1, const Vec3r& u1, const Vec3r& p2, const Vec3r& u2)
    {
        Vec3r diff = p2 - p1;
        Real uaub = u1.dot(u2);
        Real q1 = u1.dot(diff);
        Real q2 = -u2.dot(diff);
        Real d = 1 - uaub * uaub;

        if (d <= Real(1.0e-4))
        {
            return {0,0};
        }
        
        d = 1/d;
        return { (q1 + uaub * q2)*d, (uaub * q1 + q2)*d };
    }

    static int _intersectRectQuad2(Real h[2], Real p[8], Real ret[16])
    {
        // q (and r) contain nq (and nr) coordinate points for the current (and
        // chopped) polygons
        int nq = 4, nr = 0;
        Real buffer[16];
        Real* q = p;
        Real* r = ret;
        for(int dir = 0; dir <= 1; ++dir)
        {
        // direction notation: xy[0] = x axis, xy[1] = y axis
        for(int sign = -1; sign <= 1; sign += 2)
        {
            // chop q along the line xy[dir] = sign*h[dir]
            Real* pq = q;
            Real* pr = r;
            nr = 0;
            for(int i = nq; i > 0; --i)
            {
                // go through all points in q and all lines between adjacent points
                if(sign * pq[dir] < h[dir])
                {
                    // this point is inside the chopping line
                    pr[0] = pq[0];
                    pr[1] = pq[1];
                    pr += 2;
                    nr++;
                    if(nr & 8)
                    {
                        q = r;
                        goto done;
                    }
                }
                Real* nextq = (i > 1) ? pq+2 : q;
                if((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir]))
                {
                    // this line crosses the chopping line
                    pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
                    (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
                    pr[dir] = sign*h[dir];
                    pr += 2;
                    nr++;
                    if(nr & 8)
                    {
                        q = r;
                        goto done;
                    }
                }
                pq += 2;
            }
            q = r;
            r = (q == ret) ? buffer : ret;
            nq = nr;
        }
        }

        done:
        if(q != ret) memcpy(ret, q, nr*2*sizeof(Real));
        return nr;
    }

    static void _cullPoints2(int n, Real p[], int m, int i0, int iret[])
    {
        // compute the centroid of the polygon in cx,cy
        Real a, cx, cy, q;
        switch(n)
        {
            case 1:
                cx = p[0];
                cy = p[1];
                break;
            case 2:
                cx = 0.5 * (p[0] + p[2]);
                cy = 0.5 * (p[1] + p[3]);
                break;
            default:
                a = 0;
                cx = 0;
                cy = 0;

            for(int i = 0; i < n-1; ++i)
            {
                q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
                a += q;
                cx += q*(p[i*2]+p[i*2+2]);
                cy += q*(p[i*2+1]+p[i*2+3]);
            }

            q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
            if(std::abs(a+q) > std::numeric_limits<Real>::epsilon())
                a = 1/(3*(a+q));
            else
                a= 1e18f;

            cx = a*(cx + q*(p[n*2-2]+p[0]));
            cy = a*(cy + q*(p[n*2-1]+p[1]));
        }


        // compute the angle of each point w.r.t. the centroid
        Real A[8];
        for(int i = 0; i < n; ++i)
        A[i] = atan2(p[i*2+1]-cy,p[i*2]-cx);

        // search for points that have angles closest to A[i0] + i*(2*pi/m).
        int avail[8];
        for(int i = 0; i < n; ++i) avail[i] = 1;
        avail[i0] = 0;
        iret[0] = i0;
        iret++;
        const Real pi = 3.1415;
        for(int j = 1; j < m; ++j)
        {
            a = j*(2*pi/m) + A[i0];
            if (a > pi) a -= 2*pi;
            Real maxdiff= 1e9, diff;

            *iret = i0;	// iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0
            for(int i = 0; i < n; ++i)
            {
                if(avail[i])
                {
                    diff = std::abs(A[i]-a);
                    if(diff > pi) diff = 2*pi - diff;
                    if(diff < maxdiff)
                    {
                        maxdiff = diff;
                        *iret = i;
                    }
                }
            }
            avail[*iret] = 0;
            iret++;
        }
    }

};

} // namespace Collision