#include "collision/CollisionScene.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rigidbody/XPBDPlane.hpp"
#include "simobject/rod/RodCollisionSegment.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "collision/sdf/SDF.hpp"
#include "collision/sdf/SphereSDF.hpp"
#include "collision/sdf/BoxSDF.hpp"

#include "collision/helper/BoxBoxCollider.hpp"

#include <random>

#define COLLISION_TOL 1e-2      // if the distance between objects is less than this, register a collision and generate collision constraints

namespace Collision
{

// initialize static values
bool CollisionScene::_collision_table_initialized = false;
CollisionScene::CollisionFunc CollisionScene::_collision_table[static_cast<int>(ColliderType::COUNT)][static_cast<int>(ColliderType::COUNT)] = {};

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
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    // first type is a box
    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Sphere)] = [](CollisionScene* scene,void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::XPBDRigidBox*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Box)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::XPBDRigidBox*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    // first type is a rod segment
    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Sphere)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::RodCollisionSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Box)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDRigidBox*>(b), static_cast<SimObject::RodCollisionSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::RodCollisionSegment*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    // first type is a plane
    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Sphere)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Box)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRigidBox*>(b));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Plane)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDPlane*>(b));
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
    // std::cout << "\n\nCollisionScene::detectCollisions()" << std::endl;
    _new_collisions.clear();

    // run broad-phase collision detection using spatial hashing
    _spatial_hasher.hashObjects();

    // iterate through potentially colliding pairs of objects
    const SpatialHasher::CollisionPairSet& collision_pairs = _spatial_hasher.collisionPairs();
    for (auto& pair : collision_pairs)
    {
        _collision_table[static_cast<int>(pair.obj1->type)][static_cast<int>(pair.obj2->type)](this, pair.obj1->obj, pair.obj2->obj);
    }

    // iterate through planes and collide with all other objects in the scene
    const std::vector<CollisionObject>& collision_objects = _spatial_hasher.collisionObjects();
    for (auto& plane : _planes)
    {
        for (auto& collision_obj : collision_objects)
        {
            _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(collision_obj.type)](this, plane, collision_obj.obj);
        }
    }

    // return _new_collision_constraints;
    return _new_collisions;
}

bool CollisionScene::_checkJoint(const SimObject::OrientedParticle* p1, const SimObject::OrientedParticle* p2) const
{
    const SimObject::OrientedParticle* pmin = std::min(p1, p2, std::less<const SimObject::OrientedParticle*>{});
    const SimObject::OrientedParticle* pmax = std::max(p1, p2, std::less<const SimObject::OrientedParticle*>{});

    return _joint_pairs.count(std::make_pair(pmin, pmax)) > 0;
}
 

void CollisionScene::_checkCollision(CollisionScene* /* scene */, SimObject::XPBDPlane* /* plane1 */, SimObject::XPBDPlane* /* plane2 */)
{
    return;
}
void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRigidSphere* sphere)
{
    if (scene->_checkJoint(&plane->com(), &sphere->com()))
        return;

    Vec3r cp_sphere_global = sphere->com().position - sphere->radius() * plane->normal();
    Vec3r diff = cp_sphere_global - plane->com().position;
    if (diff.dot(plane->normal()) <= COLLISION_TOL)
    {
        // Collision!
        RigidRigidCollision new_collision;
        new_collision.normal = plane->normal();
        new_collision.rb1 = plane;
        new_collision.cp_local1 = Vec3r::Zero();
        new_collision.rb2 = sphere;
        new_collision.cp_local2 = sphere->com().orientation.transpose() * (cp_sphere_global - sphere->com().position);

        scene->_new_collisions.push_back(std::move(new_collision));
    }
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRigidBox* box)
{
    if (scene->_checkJoint(&plane->com(), &box->com()))
        return;

    Vec3r hsbox = box->size()/2;
    const Vec3r& pn = plane->normal();
    Real box_radius =   hsbox[0] * std::abs(box->com().orientation.col(0).dot(pn)) +
                        hsbox[1] * std::abs(box->com().orientation.col(1).dot(pn)) +
                        hsbox[2] * std::abs(box->com().orientation.col(2).dot(pn));
    Real box_proj = box->com().position.dot(pn);
    Real plane_proj = plane->com().position.dot(pn);
    if (box_proj - box_radius <= plane_proj + COLLISION_TOL)
    {

        // collision!
        Vec3r hsplane(plane->width()/2.0, plane->height()/2.0, 1e-6);
        int code = 3;   // always the Z-axis of the plane (com1) that is the separating axis

        std::vector<DetectedCollision> collisions;
        BoxBoxCollider::generateContactsForFaceSomethingCollision(
            plane, hsplane, box, hsbox,
            plane->normal(), code, collisions
        );

        scene->_new_collisions.insert(scene->_new_collisions.end(), collisions.begin(), collisions.end());
    }

}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::RodCollisionSegment* segment)
{
    if (scene->_checkJoint(&plane->com(), segment->particle1()) || scene->_checkJoint(&plane->com(), segment->particle2()))
        return;

    // const Vec3r& p1 = segment->particle1()->position;
    // const Vec3r& p2 = segment->particle2()->position;
    // Vec3r cp_p1 = p1 - segment->radius() * plane->normal();
    // Vec3r cp_p2 = p2 - segment->radius() * plane->normal();

    // Vec3r diff1 = cp_p1 - plane->com().position;
    // Vec3r diff2 = cp_p2 - plane->com().position;

    // if (diff1.dot(plane->normal()) <= COLLISION_TOL || diff2.dot(plane->normal()) <= COLLISION_TOL)
    // {
    //     for (int ind = segment->index1(); ind < segment->index2(); ind++)
    //     {
    //         Collision::RigidSegmentCollision new_collision;
    //         new_collision.beta = 0; // always create the constraint at the center of the segment for stability
    //         new_collision.cp_local_rod = segment->particle1()->orientation.transpose() * (cp_p1 - segment->particle1()->position);
    //         new_collision.normal = -plane->normal();
    //         new_collision.rb = plane;
    //         new_collision.cp_local_rb = Vec3r::Zero();
    //         new_collision.rod = segment->rod();
    //         new_collision.segment_particle1 = &segment->rod()->nodes()[ind];
    //         new_collision.segment_particle2 = &segment->rod()->nodes()[ind+1];

    //         if (ind == segment->index2()-1)
    //         {
    //             // std::cout << " Creating collision constraint at rod node " << ind+1 << std::endl;
    //             Collision::RigidSegmentCollision new_collision2 = new_collision;
    //             new_collision.beta = 1;
    //             new_collision.cp_local_rod = segment->particle2()->orientation.transpose() * (cp_p2 - segment->particle2()->position);
    //             scene->_new_collisions.push_back(std::move(new_collision2));
    //         }
    //         scene->_new_collisions.push_back(std::move(new_collision));

            
    //     }
    // }

}



void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere1, SimObject::XPBDRigidSphere* sphere2)
{
    if (sphere1 == sphere2)
        return;

    if (scene->_checkJoint(&sphere1->com(), &sphere2->com()))
        return;

    Vec3r com_diff = (sphere2->com().position - sphere1->com().position);
    Real com_sq_dist = com_diff.squaredNorm();
    Real rad_sq_dist = (sphere1->radius() + sphere2->radius())*(sphere1->radius() + sphere2->radius());
    if (com_sq_dist < rad_sq_dist + COLLISION_TOL)
    {
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
        new_collision.rb1 = sphere1;
        new_collision.rb2 = sphere2;
        new_collision.cp_local1 = r1;
        new_collision.cp_local2 = r2;
        new_collision.normal = collision_normal;
        scene->_new_collisions.push_back(std::move(new_collision));
    }
        
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidBox* box)
{
    if (scene->_checkJoint(&sphere->com(), &box->com()))
        return;

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
    if (sq_dist <= sphere->radius() * sphere->radius() + COLLISION_TOL * COLLISION_TOL)
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
        new_collision.rb1 = box;
        new_collision.rb2 = sphere;
        new_collision.cp_local1 = box_closest_point;
        new_collision.cp_local2 = cp_sphere_local;
        new_collision.normal = collision_normal;
        scene->_new_collisions.push_back(std::move(new_collision));
    }
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::RodCollisionSegment* segment)
{
    if (scene->_checkJoint(&sphere->com(), segment->particle1()) || scene->_checkJoint(&sphere->com(), segment->particle2()))
        return;

    // // project the sphere center onto the line segment, and clamp between [0,1]
    // const Vec3r& sphere_center = sphere->com().position;
    // const Vec3r& endpoint1 = segment->particle1()->position;
    // const Vec3r& endpoint2 = segment->particle2()->position;
    // Real t = Math::projectPointOntoLine(sphere_center, endpoint1, endpoint2);
    // t = std::clamp(t, 0.0, 1.0);

    // // get the closest point on the line segment
    // const Vec3r segment_cp = (1-t)*endpoint1 + t*endpoint2;

    // // find the distance
    // const Vec3r diff = sphere_center - segment_cp;
    // Real sq_dist = diff.squaredNorm();
    // Real combined_radius = sphere->radius() + segment->radius() + COLLISION_TOL;

    // if (sq_dist < combined_radius*combined_radius)
    // {
    //     // Collision!
    //     // special case: sphere center is almost on the line segment between the two rod nodes
    //     // normal should just be any perpendicular vector to the line
    //     Vec3r normal;
    //     if (sq_dist < 1e-8)
    //     {
    //         Vec3r segment_vec = endpoint2 - endpoint1;
    //         // get an arbitrary vector thats NOT parallel
    //         if (segment_vec[0] < segment_vec[1])
    //             normal = segment_vec.cross(Vec3r(1,0,0));
    //         else
    //             normal = segment_vec.cross(Vec3r(0,1,0));
    //     }
    //     else
    //     {
    //         normal = diff / std::sqrt(sq_dist);
    //     }

    //     const Vec3r cp_rod_surface = segment_cp + normal*segment->radius();

    //     const Vec3r rod_frame_o = segment_cp;
    //     const Mat3r rod_frame_R = Math::Plus_SO3(segment->particle1()->orientation, t*Math::Minus_SO3(segment->particle2()->orientation, segment->particle1()->orientation));

    //     const Vec3r cp_local_rod = rod_frame_R.transpose() * (cp_rod_surface - rod_frame_o);

    //     Vec3r sphere_cp_local = -sphere->com().orientation.transpose() * normal*sphere->radius();

    //     RigidSegmentCollision new_collision;
    //     new_collision.beta = t;
    //     new_collision.normal = normal;
    //     new_collision.cp_local_rod = cp_local_rod;
    //     new_collision.rod = segment->rod();
    //     new_collision.segment_particle1 = segment->particle1();
    //     new_collision.segment_particle2 = segment->particle2();
    //     new_collision.rb = sphere;
    //     new_collision.cp_local_rb = sphere_cp_local;
    //     scene->_new_collisions.push_back(std::move(new_collision));    
    // }
    

}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2)
{
    if (box1 == box2)
        return;

    if (scene->_checkJoint(&box1->com(), &box2->com()))
        return;

    std::vector<DetectedCollision> collisions = BoxBoxCollider::collideBoxes(box1, box1->size(), box2, box2->size());
    scene->_new_collisions.insert(scene->_new_collisions.end(), collisions.begin(), collisions.end());
}


void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box, SimObject::RodCollisionSegment* segment)
{
    if (scene->_checkJoint(&box->com(), segment->particle1()) || scene->_checkJoint(&box->com(), segment->particle2()))
        return;

    // const Vec3r& p1 = segment->particle1()->position;
    // const Vec3r& p2 = segment->particle2()->position;

    // Real beta = std::clamp(Math::projectPointOntoLine(box->com().position, p1, p2), Real(0.0), Real(1.0));
    // Vec3r cp_segment = (1-beta)*p1 + beta*p2;
    // Vec3r cp_segment_box_local = box->com().orientation.transpose() * (cp_segment - box->com().position);
    // Vec3r cp_box_local = cp_segment_box_local;
    // cp_box_local[0] = std::clamp(cp_box_local[0], -box->size()[0]/2.0, box->size()[0]/2.0);
    // cp_box_local[1] = std::clamp(cp_box_local[1], -box->size()[1]/2.0, box->size()[1]/2.0);
    // cp_box_local[2] = std::clamp(cp_box_local[2], -box->size()[2]/2.0, box->size()[2]/2.0);

    // Vec3r diff_box_local = cp_box_local - cp_segment_box_local;
    // Real dist = diff_box_local.norm();
    // if (dist <= segment->radius() + COLLISION_TOL)
    // {
    //     // collision!
    //     if (segment->size() == 1)
    //     {
    //         Vec3r normal;
    //         if (dist < 1e-12)
    //         {
    //             normal = Vec3r(1,0,0);
    //         }
    //         else
    //         {
    //             normal = box->com().orientation * diff_box_local.normalized();
    //         }

    //         const Vec3r cp_rod_surface = cp_segment + normal*segment->radius();

    //         const Vec3r rod_frame_o = cp_segment;
    //         const Mat3r rod_frame_R = Math::Plus_SO3(segment->particle1()->orientation, beta*Math::Minus_SO3(segment->particle2()->orientation, segment->particle1()->orientation));

    //         const Vec3r cp_local_rod = rod_frame_R.transpose() * (cp_rod_surface - rod_frame_o);

    //         RigidSegmentCollision new_collision;
    //         new_collision.normal = normal;
    //         new_collision.rb = box;
    //         new_collision.cp_local_rb = cp_box_local;
    //         new_collision.rod = segment->rod();
    //         new_collision.segment_particle1 = segment->particle1();
    //         new_collision.segment_particle2 = segment->particle2();
    //         new_collision.beta = beta;
    //         new_collision.cp_local_rod = cp_local_rod;
    //         scene->_new_collisions.push_back(std::move(new_collision));
    //     }
    //     else
    //     {
    //         for (int ind = segment->index1(); ind < segment->index2(); ind++)
    //         {
    //             SimObject::RodCollisionSegment subsegment(segment->rod(), ind, ind+1);
    //             _checkCollision(scene, box, &subsegment);
    //         }
    //     }
        
    // }
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::RodCollisionSegment* segment1, SimObject::RodCollisionSegment* segment2)
{
    /** Step 1: ensure segments should actually be tested for collision */
    // if the segments are the same, obviously don't test them
    if (segment1 == segment2)
        return;

    // if the segments are adjacent, don't test them
    if (segment1->particle1() == segment2->particle2() || segment1->particle2() == segment2->particle1())
        return;

    // if the segments are jointed by a joint, don't test them
    /** TODO: do this on an element level */
    if (scene->_checkJoint(segment1->particle1(), segment2->particle1()) || scene->_checkJoint(segment1->particle2(), segment2->particle1()) ||
        scene->_checkJoint(segment1->particle1(), segment2->particle2()) || scene->_checkJoint(segment1->particle2(), segment2->particle2()))
        return;

    /** Step 2: test if coarse collision segments are in collision or are close to colliding */
    const Vec3r& p1 = segment1->particle1()->position;
    const Vec3r& p2 = segment1->particle2()->position;
    const Vec3r& p3 = segment2->particle1()->position;
    const Vec3r& p4 = segment2->particle2()->position;
    auto [beta1, beta2] = Math::findClosestPointsOnLineSegments(p1, p2, p3, p4);
    
    const Vec3r cp_rod1 = p1 + beta1*(p2 - p1);
    const Vec3r cp_rod2 = p3 + beta2*(p4 - p3);

    Vec3r diff = cp_rod2 - cp_rod1;
    Real sq_dist = diff.squaredNorm();
    Real rads_sq = (segment1->radius() + segment2->radius() + COLLISION_TOL) * (segment1->radius() + segment2->radius() + COLLISION_TOL);
    if (sq_dist > rads_sq)
        return;
    
    /** Step 3: test each individual rod element within each collision segment
     * 
     */
    const std::vector<SimObject::RodElement_Base*>& rod1_elements = segment1->elements();
    const std::vector<SimObject::RodElement_Base*>& rod2_elements = segment2->elements();
    for (auto& elem1 : rod1_elements)
    {
        for (auto& elem2 : rod2_elements)
        {
            // get candidate pairs of closest positions on each rod
            std::vector<std::pair<Real,Real>> cps = SimObject::closestPointsBetweenRodElements(elem1, elem2);
            for (const auto& cp : cps)
            {
                Real s_hat1 = cp.first;
                Real s_hat2 = cp.second;

                Vec3r p1 = elem1->position(s_hat1);
                Vec3r p2 = elem2->position(s_hat2);
                Vec3r diff = (p2 - p1);

                Real sq_dist = (p2 - p1).squaredNorm();
                
                if (sq_dist < rads_sq)
                {
                    std::cout << "Rod-rod collision!" << std::endl;
                    Vec3r normal;
                    if (sq_dist < 1e-6)
                        normal = Vec3r(1,0,0);
                    else
                        normal = diff / std::sqrt(sq_dist);
                    
                    // contact points on the surface of each rod
                    const Vec3r cp_rod_surface1 = cp_rod1 + normal*segment1->radius();
                    const Vec3r cp_rod_surface2 = cp_rod2 - normal*segment2->radius();


                    const Vec3r frame1_o = cp_rod1;
                    const Mat3r frame1_R = elem1->orientation(s_hat1); 

                    const Vec3r frame2_o = cp_rod2;
                    const Mat3r frame2_R = elem2->orientation(s_hat2);

                    const Vec3r cp_local1 = frame1_R.transpose() * (cp_rod_surface1 - frame1_o);
                    const Vec3r cp_local2 = frame2_R.transpose() * (cp_rod_surface2 - frame2_o);

                    Collision::SegmentSegmentCollision new_collision;
                    new_collision.s_hat1 = s_hat1;
                    new_collision.s_hat2 = s_hat2;
                    new_collision.normal = normal;
                    new_collision.element1 = elem1;
                    new_collision.element2 = elem2;
                    new_collision.cp_local1 = cp_local1;
                    new_collision.cp_local2 = cp_local2;
                    new_collision.mu_s1 = segment1->staticFrictionCoeff();
                    new_collision.mu_d1 = segment1->dynamicFrictionCoeff();
                    new_collision.mu_s2 = segment2->staticFrictionCoeff();
                    new_collision.mu_d2 = segment2->dynamicFrictionCoeff();
                    scene->_new_collisions.push_back(std::move(new_collision));
                }
            }
        }
    }
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
        const Real beta = 2.0/(i+3);
        const Vec3r& gradient = sdf->gradient(x);
        const Real sg1 = p1.dot(gradient);
        const Real sg2 = p2.dot(gradient);
        const Real sg3 = p3.dot(gradient);

        if (sg1 < sg2 && sg1 < sg3)       s = p1;
        else if (sg2 < sg1 && sg2 < sg3)  s = p2;
        else                                s = p3;

        x = x + beta * (s - x);
        
    }

    return x;
}

} // namespace Collision