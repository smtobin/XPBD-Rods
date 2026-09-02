#include "collision/CollisionDetector.hpp"

#include "common/math.hpp"
#include "common/Algorithm.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rigidbody/XPBDPlane.hpp"
#include "simobject/rigidbody/XPBDRigidMesh.hpp"
#include "simobject/rod/RodCollisionSegment.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "collision/sdf/SDF.hpp"
#include "collision/sdf/SphereSDF.hpp"
#include "collision/sdf/BoxSDF.hpp"
#include "collision/sdf/PlaneSDF.hpp"

#include "collision/helper/BoxBoxCollider.hpp"
#include "collision/helper/RodElementCollider.hpp"

namespace Collision
{

// initialize static values
bool CollisionDetector::_collision_table_initialized = false;
CollisionDetector::CollisionFunc CollisionDetector::_collision_table[static_cast<int>(ColliderType::COUNT)][static_cast<int>(ColliderType::COUNT)] = {};

// [](void* a, void* b) {
    // CollisionDetector::_checkCollision(static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
// };
void CollisionDetector::_initCollisionTable()
{
    if (_collision_table_initialized)
        return;
    
    // first type is a sphere
    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Sphere)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Box)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidBox*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Mesh)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidMesh*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Plane)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(b), static_cast<SimObject::XPBDRigidSphere*>(a));
    };

    // first type is a box
    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Sphere)] = [](CollisionDetector* cd,void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::XPBDRigidBox*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Box)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::XPBDRigidBox*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Mesh)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::XPBDRigidMesh*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Plane)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(b), static_cast<SimObject::XPBDRigidBox*>(a));
    };

    // first type is a rod segment
    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Sphere)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::RodCollisionSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Box)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidBox*>(b), static_cast<SimObject::RodCollisionSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::RodCollisionSegment*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Mesh)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::RodCollisionSegment*>(a), static_cast<SimObject::XPBDRigidMesh*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Plane)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(b), static_cast<SimObject::RodCollisionSegment*>(a));
    };

    // first type is a plane
    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Sphere)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Box)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRigidBox*>(b));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::RodCollisionSegment*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Plane)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDPlane*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Mesh)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRigidMesh*>(b));
    };

    // first type is a mesh
    _collision_table[static_cast<int>(ColliderType::Mesh)][static_cast<int>(ColliderType::Sphere)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::XPBDRigidMesh*>(a));
    };

    _collision_table[static_cast<int>(ColliderType::Mesh)][static_cast<int>(ColliderType::Box)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidBox*>(b), static_cast<SimObject::XPBDRigidMesh*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Mesh)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::RodCollisionSegment*>(b), static_cast<SimObject::XPBDRigidMesh*>(a));
    };

    _collision_table[static_cast<int>(ColliderType::Mesh)][static_cast<int>(ColliderType::Plane)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDPlane*>(b), static_cast<SimObject::XPBDRigidMesh*>(a));
    };

    _collision_table[static_cast<int>(ColliderType::Mesh)][static_cast<int>(ColliderType::Mesh)] = [](CollisionDetector* cd, void* a, void* b) {
        CollisionDetector::_checkCollision(cd, static_cast<SimObject::XPBDRigidMesh*>(a), static_cast<SimObject::XPBDRigidMesh*>(b));
    };

    _collision_table_initialized = true;
}

CollisionDetector::CollisionDetector()
{
    _initCollisionTable();
}

CollisionDetector::CollisionDetector(const Config::CollisionSceneConfig& config)
{
    _rod_rod_collisions = config.rodRodCollisions();

    _initCollisionTable();
}

const std::vector<DetectedCollision>& CollisionDetector::detectCollisions()
{
    // and clear the current detected collisions
    _detected_collisions.clear();
    // clear the potential collisions
    _potential_collisions.clear();


    // build BVH
    _lbvh.build(_collision_objects);

    // traverse BVH for potential collisions
    _lbvh.traverseSelfIterative(_lbvh.root, _potential_collisions);

    // narrow-phase collision detection
    _narrowPhaseCollisionDetection();

    return _detected_collisions;
}

void CollisionDetector::_narrowPhaseCollisionDetection()
{
    for (const auto& potential_collision : _potential_collisions)
    {
        // potential collisions are pairs of LBVH leaf node indices, which correspond to the collision pool's sorted order
        // extract the primitive indices by indexing in the sorted order array
        unsigned a = _lbvh._sorted_order[potential_collision.first];
        unsigned b = _lbvh._sorted_order[potential_collision.second];
        CollisionObject& obj_a = _collision_objects[a];
        CollisionObject& obj_b = _collision_objects[b];

        _collision_table[static_cast<int>(obj_a.type)][static_cast<int>(obj_b.type)](this, obj_a.obj, obj_b.obj);
    }
}


bool CollisionDetector::_checkJoint(const SimObject::OrientedParticle* p1, const SimObject::OrientedParticle* p2) const
{
    const SimObject::OrientedParticle* pmin = std::min(p1, p2, std::less<const SimObject::OrientedParticle*>{});
    const SimObject::OrientedParticle* pmax = std::max(p1, p2, std::less<const SimObject::OrientedParticle*>{});

    return _joint_pairs.count(std::make_pair(pmin, pmax)) > 0;
}
 

void CollisionDetector::_checkCollision(CollisionDetector* /* cd */, SimObject::XPBDPlane* /* plane1 */, SimObject::XPBDPlane* /* plane2 */)
{
    return;
}
void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane, SimObject::XPBDRigidSphere* sphere)
{
    if (cd->_checkJoint(&plane->com(), &sphere->com()))
        return;

    Vec3r cp_sphere_global = sphere->com().position - sphere->radius() * plane->normal();
    Vec3r diff = cp_sphere_global - plane->com().position;

    Real speculative_margin = COLLISION_TOL + std::abs(sphere->com().lin_velocity.dot(plane->normal())) * COLLISION_CHECK_INTERVAL;
    if (diff.dot(plane->normal()) <= speculative_margin)
    {
        // Collision!
        RigidRigidCollision new_collision;
        new_collision.normal = plane->normal();
        new_collision.rb1 = plane;
        new_collision.cp_local1 = Vec3r::Zero();
        new_collision.rb2 = sphere;
        new_collision.cp_local2 = sphere->com().orientation.transpose() * (cp_sphere_global - sphere->com().position);

        cd->_detected_collisions.push_back(std::move(new_collision));
    }
}

void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane, SimObject::XPBDRigidBox* box)
{
    if (cd->_checkJoint(&plane->com(), &box->com()))
        return;

    Vec3r hsbox = box->size()/2;
    const Vec3r& pn = plane->normal();
    Real box_radius =   hsbox[0] * std::abs(box->com().orientation.col(0).dot(pn)) +
                        hsbox[1] * std::abs(box->com().orientation.col(1).dot(pn)) +
                        hsbox[2] * std::abs(box->com().orientation.col(2).dot(pn));
    Real box_proj = box->com().position.dot(pn);
    Real plane_proj = plane->com().position.dot(pn);

    Real speculative_margin = COLLISION_TOL + std::abs(box->com().lin_velocity.dot(pn)) * COLLISION_CHECK_INTERVAL;
    if (box_proj - box_radius <= plane_proj + speculative_margin)
    {
        // std::cout << "\nBOX-PLANE COLLISION!" << std::endl;

        // collision!
        Vec3r hsplane(plane->width()/2.0, plane->height()/2.0, 1e-6);
        int code = 3;   // always the Z-axis of the plane (com1) that is the separating axis

        std::vector<DetectedCollision> collisions;
        BoxBoxCollider::generateContactsForFaceSomethingCollision(
            plane, hsplane, box, hsbox,
            plane->normal(), code, speculative_margin,
             collisions
        );

        // std::cout << "Number of new collisions: " << collisions.size() << std::endl;

        cd->_detected_collisions.insert(cd->_detected_collisions.end(), collisions.begin(), collisions.end());
    }

}

void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane, SimObject::RodCollisionSegment* segment)
{
    if (cd->_checkJoint(&plane->com(), segment->particle1()) || cd->_checkJoint(&plane->com(), segment->particle2()))
        return;

    // for plane-rod element collision, just check each of the rod segment points individually
    for (auto& elem : segment->elements())
    {
        for (int i = 0; i < elem->numNodes(); i++)
        {
            const SimObject::OrientedParticle* node_i = elem->node(i);
            Real proj = (node_i->position - plane->com().position).dot(plane->normal());
            Real speculative_margin = COLLISION_TOL + std::abs(node_i->lin_velocity.dot(plane->normal())) * COLLISION_CHECK_INTERVAL;
            if (proj <= segment->radius() + speculative_margin)
            {
                // collision!
                // get collision point on rod centerline
                Vec3r p_rod_center = node_i->position;

                // get collision point on plane surface
                Vec3r p_rb_surface = p_rod_center - proj*plane->normal();

                // get collision point on rod surface
                Vec3r p_rod_surface = p_rod_center - segment->radius()*plane->normal();
                
                // get collision points in local frames
                Vec3r cp_local_rb = plane->com().orientation.transpose() * (p_rb_surface - plane->com().position);
                Vec3r cp_local_rod = node_i->orientation.transpose() * (p_rod_surface - p_rod_center);

                Collision::RigidSegmentCollision new_collision;
                new_collision.element = elem;
                new_collision.s_hat = Real(i) / (elem->numNodes()-1);
                new_collision.cp_local_rod = cp_local_rod;
                new_collision.rod_mu_s = segment->staticFrictionCoeff();
                new_collision.rod_mu_d = segment->dynamicFrictionCoeff();
                new_collision.rb = plane;
                new_collision.normal = -plane->normal();
                new_collision.cp_local_rb = cp_local_rb;
                cd->_detected_collisions.push_back(std::move(new_collision));
            }
        }
    }

    // PlaneSDF sdf(plane);
    // cd->_checkRigidSegmentCollision(plane, &sdf, segment);

}



void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDRigidSphere* sphere1, SimObject::XPBDRigidSphere* sphere2)
{
    if (sphere1 == sphere2)
        return;

    if (cd->_checkJoint(&sphere1->com(), &sphere2->com()))
        return;

    Vec3r com_diff = (sphere2->com().position - sphere1->com().position);
    Real com_sq_dist = com_diff.squaredNorm();
    Real rad_sq_dist = (sphere1->radius() + sphere2->radius())*(sphere1->radius() + sphere2->radius());

    Real rel_normal_speed = (sphere1->com().lin_velocity - sphere2->com().lin_velocity).dot(com_diff);
    Real speculative_margin = COLLISION_TOL + rel_normal_speed * COLLISION_CHECK_INTERVAL;
    if (std::sqrt(com_sq_dist) < std::sqrt(rad_sq_dist) + speculative_margin)
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
        // cd->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(
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
        cd->_detected_collisions.push_back(std::move(new_collision));
    }
        
}

void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidBox* box)
{
    if (cd->_checkJoint(&sphere->com(), &box->com()))
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
    Real dist = diff.norm();

    Real rel_normal_speed = (box->com().lin_velocity - sphere->com().lin_velocity).dot(diff);
    Real speculative_margin = COLLISION_TOL + rel_normal_speed * COLLISION_CHECK_INTERVAL;

    Vec3r local_collision_normal;
    if (dist <= sphere->radius() + speculative_margin)
    {
        // collision!
        
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
        // cd->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
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
        cd->_detected_collisions.push_back(std::move(new_collision));
    }
}

void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDRigidSphere* sphere, SimObject::RodCollisionSegment* segment)
{
    if (cd->_checkJoint(&sphere->com(), segment->particle1()) || cd->_checkJoint(&sphere->com(), segment->particle2()))
        return;

    SphereSDF sdf(sphere);
    cd->_checkRigidSegmentCollision(sphere, &sdf, segment);
}

void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2)
{
    if (box1 == box2)
        return;

    if (cd->_checkJoint(&box1->com(), &box2->com()))
        return;

    std::vector<DetectedCollision> collisions = BoxBoxCollider::collideBoxes(box1, box1->size(), box2, box2->size());
    cd->_detected_collisions.insert(cd->_detected_collisions.end(), collisions.begin(), collisions.end());
}


void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::XPBDRigidBox* box, SimObject::RodCollisionSegment* segment)
{

    if (cd->_checkJoint(&box->com(), segment->particle1()) || cd->_checkJoint(&box->com(), segment->particle2()))
    {
        return;
    }

    BoxSDF sdf(box);
    cd->_checkRigidSegmentCollision(box, &sdf, segment);
}

void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::RodCollisionSegment* segment1, SimObject::RodCollisionSegment* segment2)
{
    if (!cd->_rod_rod_collisions)
        return;

    /** Step 1: ensure segments should actually be tested for collision */
    // if the segments are the same, obviously don't test them
    if (segment1 == segment2)
        return;

    // if the segments are adjacent, don't test them
    if (segment1->particle1() == segment2->particle2() || segment1->particle2() == segment2->particle1())
        return;

    // if the segments are jointed by a joint, don't test them
    /** TODO: do this on an element level */
    if (cd->_checkJoint(segment1->particle1(), segment2->particle1()) || cd->_checkJoint(segment1->particle2(), segment2->particle1()) ||
        cd->_checkJoint(segment1->particle1(), segment2->particle2()) || cd->_checkJoint(segment1->particle2(), segment2->particle2()))
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
    Real dist = diff.norm();

    Vec3r approx_seg_lin_vel1 = (1-beta1) * segment1->particle1()->lin_velocity + beta1 * segment1->particle2()->lin_velocity;
    Vec3r approx_seg_lin_vel2 = (1-beta2) * segment2->particle1()->lin_velocity + beta2 * segment2->particle2()->lin_velocity;
    Vec3r approx_seg_pos1 = (1-beta1) * segment1->particle1()->position + beta1 * segment1->particle2()->position;
    Vec3r approx_seg_pos2 = (1-beta2) * segment2->particle1()->position + beta2 * segment2->particle2()->position;

    Real rel_normal_speed = (approx_seg_lin_vel1 - approx_seg_lin_vel2).dot(approx_seg_pos2 - approx_seg_pos1);
    Real speculative_margin = COLLISION_TOL + rel_normal_speed * COLLISION_CHECK_INTERVAL;
    if (dist > segment1->radius() + segment2->radius() + speculative_margin)
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
            std::vector<std::pair<Real,Real>> cps = RodElementCollider::closestPointsBetweenRodElements(elem1, elem2);
            for (const auto& cp : cps)
            {
                Real s_hat1 = cp.first;
                Real s_hat2 = cp.second;

                Vec3r p1 = elem1->position(s_hat1);
                Vec3r p2 = elem2->position(s_hat2);
                Vec3r diff = (p2 - p1);

                Real dist = (p2 - p1).norm();
                

                Vec3r seg_lin_vel1 = elem1->linearVelocity(s_hat1);
                Vec3r seg_lin_vel2 = elem2->linearVelocity(s_hat2);

                Real rel_normal_speed = (seg_lin_vel1 - seg_lin_vel2).dot(p2 - p1);
                Real speculative_margin = COLLISION_TOL + rel_normal_speed * COLLISION_CHECK_INTERVAL;
                if (dist < segment1->radius() + segment2->radius() + speculative_margin)
                {
                    // std::cout << "Rod-rod collision!" << std::endl;
                    Vec3r normal;
                    if (dist < 1e-6)
                        normal = Vec3r(1,0,0);
                    else
                        normal = diff / dist;
                    
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
                    cd->_detected_collisions.push_back(std::move(new_collision));
                }
            }
        }
    }
}

void CollisionDetector::_checkCollision(CollisionDetector* /* cd */, SimObject::XPBDPlane* /* plane */, SimObject::XPBDRigidMesh* /* mesh */)
{

}

void CollisionDetector::_checkCollision(CollisionDetector* /* cd */, SimObject::XPBDRigidSphere* /* sphere */, SimObject::XPBDRigidMesh* /* mesh */)
{

}

void CollisionDetector::_checkCollision(CollisionDetector* /* cd */, SimObject::XPBDRigidBox* /* box */, SimObject::XPBDRigidMesh* /* mesh */)
{

}

void CollisionDetector::_checkCollision(CollisionDetector* cd, SimObject::RodCollisionSegment* segment, SimObject::XPBDRigidMesh* mesh)
{
    // std::cout << "Potential segment-mesh collision!" << std::endl;
    cd->_checkRigidSegmentCollision(mesh, &mesh->sdf(), segment);
}

void CollisionDetector::_checkCollision(CollisionDetector* /* cd */, SimObject::XPBDRigidMesh* /* mesh1 */, SimObject::XPBDRigidMesh* /* mesh2 */)
{

}

void CollisionDetector::_checkRigidSegmentCollision(SimObject::XPBDRigidBody_Base* rb, const SDF* rb_sdf, SimObject::RodCollisionSegment* segment)
{
    // if there are multiple elements in the collision segment, approximate the entire collision segment as a linear element
    // as an initial coarse pass
    if (segment->elements().size() > 1)
    {
        
        // first, use coarse, linear approximation of collision segment
        auto [s_approx, dist_approx] = RodElementCollider::closestPointBetweenLineAndSDF(segment->particle1()->position, segment->particle2()->position, rb_sdf);
        
        Vec3r approx_seg_lin_vel = (1-s_approx) * segment->particle1()->lin_velocity + s_approx * segment->particle2()->lin_velocity;
        Vec3r approx_seg_pos = (1-s_approx) * segment->particle1()->position + s_approx * segment->particle2()->position;
        Real rel_normal_speed = (approx_seg_lin_vel - rb->com().lin_velocity).dot(rb->com().position - approx_seg_pos);
        Real speculative_margin = COLLISION_TOL + rel_normal_speed * COLLISION_CHECK_INTERVAL;

        if (dist_approx > 2*segment->radius() + speculative_margin)
            return;
    }

    // iterate through each element in the collision segment and check ofr collision with the SDF
    for (auto& elem : segment->elements())
    {
        // get the "deepest penetrating" point on the rod element centerline
        auto [s, dist] = RodElementCollider::closestPointBetweenRodElementAndSDF(elem, rb_sdf);
        
        Vec3r seg_vel = elem->linearVelocity(s);
        Vec3r seg_pos = elem->position(s);
        Real rel_normal_speed = (seg_vel - rb->com().lin_velocity).dot(rb->com().position - seg_pos);
        Real speculative_margin = COLLISION_TOL + rel_normal_speed * COLLISION_CHECK_INTERVAL;

        if (dist <= segment->radius() + speculative_margin)
        {
            // collision!
            // get collision point on rod centerline
            Vec3r p_rod_center = elem->position(s);
            

            // get collision point on rigid body surface
            Vec3r sdf_grad = rb_sdf->gradient(p_rod_center);
            Vec3r p_rb_surface = p_rod_center - dist*sdf_grad;

            // get collision point on rod surface
            Vec3r p_rod_surface = p_rod_center - segment->radius()*sdf_grad;
            
            // get collision points in local frames
            Vec3r cp_local_rb = rb->com().orientation.transpose() * (p_rb_surface - rb->com().position);

            Mat3r rod_R = elem->orientation(s);
            Vec3r cp_local_rod = rod_R.transpose() * (p_rod_surface - p_rod_center);

            Collision::RigidSegmentCollision new_collision;
            new_collision.element = elem;
            new_collision.s_hat = s;
            new_collision.cp_local_rod = cp_local_rod;
            new_collision.rod_mu_s = segment->staticFrictionCoeff();
            new_collision.rod_mu_d = segment->dynamicFrictionCoeff();
            new_collision.rb = rb;
            new_collision.normal = -sdf_grad;
            new_collision.cp_local_rb = cp_local_rb;
            _detected_collisions.push_back(std::move(new_collision));

            // std::cout << "Rigid-segment collision! Rigid body name: " << rb->name() << " dist: " << dist << " speculative margin: " << speculative_margin << " radius: " << segment->radius() <<std::endl;
        }
    }
}

// bool CollisionDetector::_triangleSDF_CCD(Sim::SimulationContext& ctx, unsigned triangle, unsigned rb, Real dt, Vec3r& normal, Vec3r& cp_barys, Vec3r& cp_rb_local)
// {
//     unsigned params_idx = ctx.collision_pool.shapeParamsIndex(rb);      // index in the SDF pool for the rigid body
//     unsigned rb_idx = ctx.collision_pool.particle_indices[rb][0];   // particle index of the rigid body COM
//     const CollisionShapeParams& sdf_params = ctx.collision_pool.shape_params_pool.shape_params[params_idx];     // SDF parameters

//     // extract quantities for rigid body
//     const Vec3r& rb_pos = ctx.particles.positions[rb_idx];
//     const Quaternion& rb_rot = ctx.particles.rotation(rb_idx);
//     const Vec3r& rb_lin_vel = ctx.particles.velocities[rb_idx];
//     const Vec3r& rb_ang_vel = ctx.particles.angularVelocity(rb_idx);    // note: body-frame angular velocity

//     // evolves rigid body position forward by t
//     auto rigid_body_pos = [&](Real t)
//     {
//         return rb_pos + rb_lin_vel * t;
//     };

//     // evolves rigid body rotation forward by t
//     auto rigid_body_rot = [&](Real t)
//     {
//         return rb_rot * Math::Exp_s3(rb_ang_vel*t);
//     };

//     // computes the linear velocity of a point in space if it were attached to the rigid body
//     auto lin_vel_on_rigid_body = [&](const Vec3r& xt, Real t)
//     {
//         const Vec3r rb_ang_vel_global = rigid_body_rot(t) * rb_ang_vel;
//         return rb_lin_vel + rb_ang_vel_global.cross(xt - rigid_body_pos(t));
//     };

//     // computes the relative velocity of a point in space with respect to the rigid body (in the rigid body's local frame)
//     auto rel_vel_wrt_rigid_body = [&](const Vec3r& xt, const Vec3r& vt, Real t)
//     {
//         const Vec3r v_rel_global = vt - lin_vel_on_rigid_body(xt, t);
//         return rigid_body_rot(t).inverse() * v_rel_global;
//     };

//     // transforms a position into the rigid body local frame
//     auto local_pos_wrt_rigid_body = [&](const Vec3r& xt, Real t)
//     {
//         return rigid_body_rot(t).inverse() * (xt - rigid_body_pos(t));
//     };

//     // extract quantities for triangle
//     const auto& tri_indices = ctx.collision_pool.particle_indices[triangle];
//     const Vec3r& tri1 = ctx.particles.positions[tri_indices[0]];
//     const Vec3r& tri2 = ctx.particles.positions[tri_indices[1]];
//     const Vec3r& tri3 = ctx.particles.positions[tri_indices[2]];
//     const Vec3r& tri_vel1 = ctx.particles.velocities[tri_indices[0]];
//     const Vec3r& tri_vel2 = ctx.particles.velocities[tri_indices[1]];
//     const Vec3r& tri_vel3 = ctx.particles.velocities[tri_indices[2]];

//     /** Find initial iterate */
//     // helper for evaluating which vertex of the triangle to start at
//     // use the vertex that minimizes vi^T * gradSDF(si) where si are the triangle vertices and vi the corresponding linear velocities
//     auto evaluate_initial_iterate = [&](const Vec3r& si, const Vec3r& vi)
//     {
//         const Vec3r rel_vel = rel_vel_wrt_rigid_body(si, vi, 0);
//         const Vec3r rel_pos = local_pos_wrt_rigid_body(si, 0);

//         // vi^T * gradSDF(si) (everything expressed in SDF local frame)
//         return rel_vel.dot(SDF::gradient(sdf_params, rel_pos));
//     };

//     // evaluate each vertex
//     Real eval1 = evaluate_initial_iterate(tri1, tri_vel1);
//     Real eval2 = evaluate_initial_iterate(tri2, tri_vel2);
//     Real eval3 = evaluate_initial_iterate(tri3, tri_vel3);

//     // optimization variables
//     Real t_start = 0, t_end = dt;  // time interval
//     Real t = 0; // time
//     Real u,v,w; // barycentric coordinates on triangle

//     // vertex 1 is the initial iterate
//     if (eval1 < eval2 && eval1 < eval3)         { u = 1; v = 0; w = 0; }
//     // vertex 2 is the initial iterate
//     else if (eval2 < eval1 && eval2 < eval3)    { u = 0; v = 1; w = 0; }
//     // vertex 3 is the initial iterate
//     else                                        { u = 0; v = 0; w = 1; }


//     /** Spatio-temporal optimization */
//     // helper that interpolates the barycentric position in time
//     // note: this is still the global position - will need to convert to local SDF frame
//     auto barycentric_interpolate = [&](Real u, Real v, Real w, Real t)
//     {
//         return u*(tri1 + tri_vel1*t) + v*(tri2 + tri_vel2*t) + w*(tri3 + tri_vel3*t);
//     };
//     // helper for velocity interpolation of point on triangle (global frame)
//     auto barycentric_interpolate_velocity = [&](Real u, Real v, Real w)
//     {
//         return u*tri_vel1 + v*tri_vel2 + w*tri_vel3;
//     };
//     // evaluate the SDF at a given time
//     auto signed_distance_at_time = [&](Real t)
//     {
//         Vec3r x_global = barycentric_interpolate(u, v, w, t);
//         Vec3r x = local_pos_wrt_rigid_body(x_global, t);
//         return SDF::evaluate(sdf_params, x);
//     };
//     // evaluate the abs of the SDF at a given time
//     auto unsigned_distance_at_time = [&](Real t)
//     {
//         return std::abs(signed_distance_at_time(t));
//     };

//     // evaluate SDF at a given point - assumes positions are in rigid body local frame
//     auto signed_distance_at_point = [&](const Vec3r& x_local)
//     {
//         return SDF::evaluate(sdf_params, x_local);
//     };

//     constexpr unsigned MAX_ITER = 16;
//     constexpr Real TOL = 1e-6;
//     Real sdf_xti;   // latest SDF evaluation
//     Vec3r grad_xti; // latest SDF gradient
//     Vec3r x_ti;     // latest contact point on triangle
//     for (unsigned iter = 0; iter < MAX_ITER; iter++)
//     {
//         /** Solve temporal subproblem */
//         Real t_old = t;
//         Vec3r x_ti_global = barycentric_interpolate(u, v, w, t);
//         Vec3r v_ti_global = barycentric_interpolate_velocity(u, v, w);
//         x_ti = local_pos_wrt_rigid_body(x_ti_global, t);
//         Vec3r v_ti = rel_vel_wrt_rigid_body(x_ti_global, v_ti_global, t);

//         sdf_xti = SDF::evaluate(sdf_params, x_ti);
//         if (sdf_xti <= 0)
//         {
//             // update the interval
//             t_end = std::min(t, t_end);
//             t = Algorithm::goldenSectionSearch(t_start, t, unsigned_distance_at_time);
//         }
//         else
//         {
//             Real v_dot_grad = v_ti.dot(SDF::gradient(sdf_params, x_ti));
//             Real d = v_dot_grad > 0 ? -1 : 1;
//             if (d < 0)
//                 t = Algorithm::goldenSectionSearch(t_start, t, signed_distance_at_time);
//             else
//                 t = Algorithm::goldenSectionSearch(t, t_end, signed_distance_at_time);
//         }

//         /** Solve spatial subproblem */
//         x_ti_global = barycentric_interpolate(u, v, w, t);
//         v_ti_global = barycentric_interpolate_velocity(u, v, w);
//         x_ti = local_pos_wrt_rigid_body(x_ti_global, t);
//         v_ti = rel_vel_wrt_rigid_body(x_ti_global, v_ti_global, t);

//         sdf_xti = SDF::evaluate(sdf_params, x_ti);
//         grad_xti = SDF::gradient(sdf_params, x_ti);
//         if (sdf_xti <= 0)
//             t_end = std::min(t, t_end);
        
//         // find support vertex
//         Vec3r tri1_localt = local_pos_wrt_rigid_body(tri1 + tri_vel1*t, t);
//         Vec3r tri2_localt = local_pos_wrt_rigid_body(tri2 + tri_vel2*t, t);
//         Vec3r tri3_localt = local_pos_wrt_rigid_body(tri3 + tri_vel3*t, t);
//         Real s1 = tri1_localt.dot(grad_xti);
//         Real s2 = tri2_localt.dot(grad_xti);
//         Real s3 = tri3_localt.dot(grad_xti);
//         Vec3r s;
//         if (s1 < s2 && s1 < s3)         s = tri1_localt;
//         else if (s2 < s1 && s2 < s3)    s = tri2_localt;
//         else                            s = tri3_localt;

//         Vec3r x_new = Algorithm::goldenSectionSearch(x_ti, s, signed_distance_at_point);
//         Vec3r barys = Math::barycentricCoordinates(x_new, tri1_localt, tri2_localt, tri3_localt);
//         u = barys[0]; v = barys[1]; w = barys[2];

//         // see if we can exit early
//         if (std::abs(t_old - t) < TOL && (x_new - x_ti).norm() < TOL)
//             break;
//     }

//     cp_barys = Vec3r(u, v, w);
//     cp_rb_local = x_ti - sdf_xti*grad_xti;
//     normal = rigid_body_rot(t) * grad_xti;

//     // std::cout << "Best t: " << t << std::endl;
//     // std::cout << "SDF @ t: " << signed_distance_at_time(t) << std::endl;

//     return sdf_xti < ctx.params.collision_margin;
    

// }

} // namespace Collision