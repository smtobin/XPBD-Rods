#include "collision/CollisionScene.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rod/XPBDRodSegment.hpp"

#include "collision/sdf/SDF.hpp"
#include "collision/sdf/SphereSDF.hpp"
#include "collision/sdf/BoxSDF.hpp"

namespace Collision
{

// initialize static values
bool CollisionScene::_collision_table_initialized = false;
CollisionScene::CollisionFunc CollisionScene::_collision_table[3][3] = {};

// [](void* a, void* b) {
    // CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
// };
void CollisionScene::_initCollisionTable()
{
    if (_collision_table_initialized)
        return;
    
    // first type is a sphere
    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Sphere)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Box)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidBox*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRodSegment*>(b));
    };

    // first type is a box
    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Sphere)] = [](CollisionScene* scene,void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::XPBDRigidBox*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Box)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::XPBDRigidBox*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::XPBDRodSegment*>(b));
    };

    // first type is a rod segment
    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Sphere)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::XPBDRodSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Box)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidBox*>(b), static_cast<SimObject::XPBDRodSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRodSegment*>(a), static_cast<SimObject::XPBDRodSegment*>(b));
    };

    _collision_table_initialized = true;
}

CollisionScene::CollisionScene()
    : _spatial_hasher(0.5, 14909)
{
    _initCollisionTable();
}

CollisionScene::CollisionScene(Real grid_size, int num_buckets)
    : _spatial_hasher(grid_size, num_buckets)
{
    _initCollisionTable();
}

// const XPBDCollisionConstraints_Container& CollisionScene::detectCollisions()
const std::vector<DetectedCollision>& CollisionScene::detectCollisions()
{
    _new_collisions.clear();

    // run broad-phase collision detection using spatial hashing
    _spatial_hasher.hashObjects();

    // iterate through potentially colliding pairs of objects
    const SpatialHasher::CollisionPairSet& collision_pairs = _spatial_hasher.collisionPairs();
    for (auto& pair : collision_pairs)
    {
        _collision_table[static_cast<int>(pair.obj1->type)][static_cast<int>(pair.obj2->type)](this, pair.obj1->obj, pair.obj2->obj);
    }

    // return _new_collision_constraints;
    return _new_collisions;
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere1, SimObject::XPBDRigidSphere* sphere2)
{
    if (sphere1 == sphere2)
        return;

    std::cout << "Potential collision between two spheres!" << std::endl;

    Vec3r com_diff = (sphere2->com().position - sphere1->com().position);
    Real com_sq_dist = com_diff.squaredNorm();
    Real rad_sq_dist = (sphere1->radius() + sphere2->radius())*(sphere1->radius() + sphere2->radius());
    if (com_sq_dist < rad_sq_dist)
    {
        // collision!
        std::cout << "\n\n\n COLLISION BETWEEN SPHERES!\n\n\n" << std::endl;
        // collision normal points from sphere 1 -> sphere 2
        Vec3r collision_normal;
        if (com_sq_dist < CONSTRAINT_EPS)
            collision_normal = Vec3r(1, 0, 0);
        else
            collision_normal = com_diff / std::sqrt(com_sq_dist);
        
        // Vec3r cp1 = sphere1->com() + collision_normal * sphere1->radius();
        Vec3r r1 = sphere1->com().orientation.transpose() * collision_normal * sphere1->radius();
        Vec3r r2 = sphere2->com().orientation.transpose() * -collision_normal * sphere2->radius();
        // Vec3r cp2 = sphere2->com() - collision_normal * sphere2->radius();

        // create collision constraint
        // scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(
        //     &sphere1->com(), r1, 
        //     &sphere2->com(), r2, 
        //     collision_normal
        // );
        RigidRigidCollision new_collision;
        new_collision.particle1 = &sphere1->com();
        new_collision.particle2 = &sphere2->com();
        new_collision.cp_local1 = r1;
        new_collision.cp_local2 = r2;
        new_collision.normal = collision_normal;
        scene->_new_collisions.push_back(std::move(new_collision));
    }
        
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidBox* box)
{
    std::cout << "Potential collision between a sphere and a box!" << std::endl;
    // find closest point on box to sphere center
    // first, transform sphere center into box local frame
    const Vec3r sphere_local = box->com().orientation.transpose() * (sphere->com().position - box->com().position);
    Vec3r box_closest_point(
        std::clamp(sphere_local[0], -box->size()[0]/2, box->size()[0]/2),
        std::clamp(sphere_local[1], -box->size()[1]/2, box->size()[1]/2),
        std::clamp(sphere_local[2], -box->size()[2]/2, box->size()[2]/2)
    );

    // vector (in box frame) from closest point to sphere center
    const Vec3r diff = sphere_local - box_closest_point;
    Real sq_dist = diff.squaredNorm();

    Vec3r local_collision_normal;
    if (sq_dist <= sphere->radius() * sphere->radius())
    {
        // collision!
        std::cout << "\n\n\n COLLISION BETWEEN SPHERE AND BOX!\n\n\n" << std::endl;

        Real dist = std::sqrt(sq_dist);
        
        // edge case: sphere center inside box
        // because of the clamp operation, the distance to the closest point in the box will be 0
        if (dist < 1e-6)
        {
            // find the axis with the minimum penetration
            const Vec3r penetrations = box->size()/2 - sphere_local.cwiseAbs();
            if (penetrations[0] < penetrations[1] && penetrations[0] < penetrations[2])
            {
                // minimum penetration along local x axis
                local_collision_normal = Vec3r(sphere_local[0] > 0 ? 1 : -1, 0, 0);
                box_closest_point[0] = box->size()[0]/2 * (sphere_local[0] > 0 ? 1 : -1); 
            }
            else if (penetrations[1] < penetrations[2])
            {
                // minimum penetration is along local y axis
                local_collision_normal = Vec3r(0, sphere_local[1] > 0 ? 1 : -1, 0);
                box_closest_point[1] = box->size()[1]/2 * (sphere_local[1] > 0 ? 1 : -1);
            }
            else
            {
                // minimum penetration is along local z axis
                local_collision_normal = Vec3r(0, 0, sphere_local[2] > 0 ? 1 : -1);
                box_closest_point[2] = box->size()[2]/2 * (sphere_local[2] > 0 ? 1 : -1);
            }
        }
        else
        {
            // normal case: sphere center outside box
            local_collision_normal = diff / dist;
        }

        Vec3r collision_normal = box->com().orientation * local_collision_normal;
        Vec3r cp_sphere_local = -sphere->com().orientation.transpose() * collision_normal * sphere->radius();

        // create collision constraint
        // scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
        //     &box->com(), box_closest_point,
        //     &sphere->com(), cp_sphere_local,
        //     collision_normal
        // );
        RigidRigidCollision new_collision;
        new_collision.particle1 = &box->com();
        new_collision.particle2 = &sphere->com();
        new_collision.cp_local1 = box_closest_point;
        new_collision.cp_local2 = cp_sphere_local;
        new_collision.normal = collision_normal;
        scene->_new_collisions.push_back(std::move(new_collision));
    }
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRodSegment* segment)
{
    std::cout << "Potential collision between a sphere and rod segment!" << std::endl;

    // project the sphere center onto the line segment, and clamp between [0,1]
    const Vec3r& sphere_center = sphere->com().position;
    const Vec3r& endpoint1 = segment->particle1()->position;
    const Vec3r& endpoint2 = segment->particle2()->position;
    Real t = Math::projectPointOntoLine(sphere_center, endpoint1, endpoint2);
    t = std::clamp(t, 0.0, 1.0);

    // get the closest point on the line segment
    const Vec3r segment_cp = (1-t)*endpoint1 + t*endpoint2;

    // find the distance
    const Vec3r diff = sphere_center - segment_cp;
    Real sq_dist = diff.squaredNorm();
    

}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2)
{
    if (box1 == box2)
        return;

    std::cout << "Potential collision between two boxes!" << std::endl;

    // separating axis theorem - look for axes where projections don't overlap
    // 15 potential separating axes:
    //  - 3 face normals of box 1
    //  - 3 face normals of box 2
    //  - 9 edge cross products (3 edges of box 1 x 3 edges of box 2)
    
    Real min_penetration = std::numeric_limits<Real>::max();
    Vec3r normal = box1->com().orientation.col(0);
    Vec3r center_diff = box2->com().position - box1->com().position;
    bool flip_normal = false;
    int code;

    auto test_axis = [&](const Vec3r& axis, int cur_code) -> bool
    {
        Real proj1 =    box1->size()[0]/2 * std::abs(box1->com().orientation.col(0).dot(axis)) +
                        box1->size()[1]/2 * std::abs(box1->com().orientation.col(1).dot(axis)) +
                        box1->size()[2]/2 * std::abs(box1->com().orientation.col(2).dot(axis));
        Real proj2 =    box2->size()[0]/2 * std::abs(box2->com().orientation.col(0).dot(axis)) +
                        box2->size()[1]/2 * std::abs(box2->com().orientation.col(1).dot(axis)) +
                        box2->size()[2]/2 * std::abs(box2->com().orientation.col(2).dot(axis));
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
        if (!test_axis(box1->com().orientation.col(i), i+1))
            return;

        if (!test_axis(box2->com().orientation.col(i), i+4))
            return;
    }

    // edge cross products
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Vec3r axis = box1->com().orientation.col(i).cross(box2->com().orientation.col(j));
            Real sq_len = axis.squaredNorm();
            // skip near-parallel edges
            if (sq_len < 1e-6)
                continue;

            axis = axis / std::sqrt(sq_len);

            if (!test_axis(axis, 7 + 3*i+j))
                return;
        }
    }

    // all axes overlap ==> collision!
    std::cout << "BOX-BOX COLLISION! code " << code << std::endl;

    // always want the normal pointing from box1 to box2
    // if the minimum separating axis dotted with the vector (box2_pos - box1_pos) was negative, we need to flip the normal
    if (flip_normal)
        normal = -normal;
    
    /** Compute contact point(s) */

    // Edge-edge contact!
    if (code > 6)
    {
        // find point p1 on the intersecting edge of box 1
        Vec3r p1(box1->com().position);
        for (int k = 0; k < 3; k++)
        {
            int sign = (box1->com().orientation.col(k).dot(normal) > 0) ? 1 : -1;
            p1 += box1->com().orientation.col(k) * (box1->size()[k]/2.0 * sign);
        }

        // find point p2 on the intersecting edge of box 2
        Vec3r p2(box2->com().position);
        for (int k = 0; k < 3; k++)
        {
            int sign = (box2->com().orientation.col(k).dot(normal) > 0) ? -1 : 1;
            p2 += box2->com().orientation.col(k) * (box2->size()[k]/2.0 * sign);
        }

        // directions of intersecting edges for each box ==> just the columns of the rotation matrix
        Vec3r u1(box1->com().orientation.col((code - 7)/3));
        Vec3r u2(box2->com().orientation.col((code - 7)%3));

        // find the closest point between the lines defined by p1 in the direction of u1, and p2 in the direction of u2
        // returns two interpolation parameters that will give us the contact points
        const auto [alpha, beta] = _closestPointBetweenLines(p1, u1, p2, u2);

        // p1 and p2 become the contact points (move along each respective edge to get to the contact point)
        p1 += alpha * u1;
        p2 += beta * u2;

        Vec3r p1_local = box1->com().orientation.transpose() * (p1 - box1->com().position);
        Vec3r p2_local = box2->com().orientation.transpose() * (p2 - box2->com().position);

        RigidRigidCollision new_collision;
        new_collision.cp_local1 = p1_local;
        new_collision.cp_local2 = p2_local;
        new_collision.normal = normal;
        new_collision.particle1 = &box1->com();
        new_collision.particle2 = &box2->com();
        scene->_new_collisions.push_back(std::move(new_collision));

        return;
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
        R1 = box1->com().orientation;
        R2 = box2->com().orientation;
        p1 = box1->com().position;
        p2 = box2->com().position;
        halfsize1 = box1->size()/2;
        halfsize2 = box2->size()/2;

        com1 = &(box1->com());
        com2 = &(box2->com());
    }
    // normal corresponds to a face on box 2
    else
    {
        R1 = box2->com().orientation;
        R2 = box1->com().orientation;
        p1 = box2->com().position;
        p2 = box1->com().position;
        halfsize1 = box2->size()/2;
        halfsize2 = box1->size()/2;

        com1 = &(box2->com());
        com2 = &(box1->com());
    }

    // normal vector of the reference face
    Vec3r normal2 = normal;
    if (code > 4)
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
    if(n_intersect < 1) { std::cout << "LINE 513 assert(0);"<< std::endl; return; } // this should never happen

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
    if(cnum < 1) { assert(0); return; } // this should never happen

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
            
            // scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
            //     com1, cp1_local,
            //     com2, cp2_local,
            //     normal
            // );

            RigidRigidCollision new_collision;
            new_collision.cp_local1 = cp1_local;
            new_collision.cp_local2 = cp2_local;
            new_collision.normal = normal;
            new_collision.particle1 = com1;
            new_collision.particle2 = com2;
            scene->_new_collisions.push_back(std::move(new_collision));
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
            
            // scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
            //     com1, cp1_local,
            //     com2, cp2_local,
            //     normal
            // );

            RigidRigidCollision new_collision;
            new_collision.cp_local1 = cp1_local;
            new_collision.cp_local2 = cp2_local;
            new_collision.normal = normal;
            new_collision.particle1 = com1;
            new_collision.particle2 = com2;
            scene->_new_collisions.push_back(std::move(new_collision));
        }
    }

    std::cout << "Number of contacts: " << cnum << std::endl;
}

std::pair<Real, Real> CollisionScene::_closestPointBetweenLines(const Vec3r& p1, const Vec3r& u1, const Vec3r& p2, const Vec3r& u2)
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

int CollisionScene::_intersectRectQuad2(Real h[2], Real p[8], Real ret[16])
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

void CollisionScene::_cullPoints2(int n, Real p[], int m, int i0, int iret[])
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


void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box, SimObject::XPBDRodSegment* segment)
{
    std::cout << "Potential collision between a box and a rod segment!" << std::endl;
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRodSegment* segment1, SimObject::XPBDRodSegment* segment2)
{
    if (segment1 == segment2)
        return;

    std::cout << "Potential collision between two rod segments!" << std::endl;
}

Vec3r CollisionScene::_frankWolfe(const SDF* sdf, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3)
{
    // find starting iterate - the triangle vertex with the smallest value of SDF
    const Real d_p1 = sdf->evaluate(p1);
    const Real d_p2 = sdf->evaluate(p2);
    const Real d_p3 = sdf->evaluate(p3);

    Vec3r x;
    if (d_p1 <= d_p2 && d_p1 <= d_p3)       x = p1;
    else if (d_p2 <= d_p1 && d_p2 <= d_p3)  x = p2;
    else                                    x = p3;

    Vec3r s;
    for (int i = 0; i < 10; i++)
    {
        const Real alpha = 2.0/(i+3);
        const Vec3r& gradient = sdf->gradient(x);
        const Real sg1 = p1.dot(gradient);
        const Real sg2 = p2.dot(gradient);
        const Real sg3 = p3.dot(gradient);

        if (sg1 < sg2 && sg1 < sg3)       s = p1;
        else if (sg2 < sg1 && sg2 < sg3)  s = p2;
        else                                s = p3;

        x = x + alpha * (s - x);
        
    }

    return x;
}

} // namespace Collision