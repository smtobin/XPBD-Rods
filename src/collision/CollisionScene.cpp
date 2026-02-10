#include "collision/CollisionScene.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rigidbody/XPBDPlane.hpp"
#include "simobject/rod/XPBDRodSegment.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "collision/sdf/SDF.hpp"
#include "collision/sdf/SphereSDF.hpp"
#include "collision/sdf/BoxSDF.hpp"

#include "collision/helper/BoxBoxCollider.hpp"

#include <random>

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

    // first type is a plane
    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Sphere)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::Box)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRigidBox*>(b));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Plane)][static_cast<int>(ColliderType::RodSegment)] = [](CollisionScene* scene, void* a, void* b) {
        CollisionScene::_checkCollision(scene, static_cast<SimObject::XPBDPlane*>(a), static_cast<SimObject::XPBDRodSegment*>(b));
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

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane1, SimObject::XPBDPlane* plane2)
{
    return;
}
void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRigidSphere* sphere)
{
    Vec3r cp_sphere_global = sphere->com().position - sphere->radius() * plane->normal();
    Vec3r diff = cp_sphere_global - plane->com().position;
    if (diff.dot(plane->normal()) <= 0)
    {
        // Collision!
        RigidRigidCollision new_collision;
        new_collision.normal = plane->normal();
        new_collision.particle1 = &plane->com();
        new_collision.cp_local1 = Vec3r::Zero();
        new_collision.particle2 = &sphere->com();
        new_collision.cp_local2 = sphere->com().orientation.transpose() * (cp_sphere_global - sphere->com().position);

        scene->_new_collisions.push_back(std::move(new_collision));
    }
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRigidBox* box)
{
    Vec3r hsbox = box->size()/2;
    const Vec3r& pn = plane->normal();
    Real box_radius =   hsbox[0] * std::abs(box->com().orientation.col(0).dot(pn)) +
                        hsbox[1] * std::abs(box->com().orientation.col(1).dot(pn)) +
                        hsbox[2] * std::abs(box->com().orientation.col(2).dot(pn));
    Real box_proj = box->com().position.dot(pn);
    Real plane_proj = plane->com().position.dot(pn);
    if (box_proj - box_radius <= plane_proj)
    {

        // collision!
        Vec3r hsplane(plane->width()/2.0, plane->height()/2.0, 1e-6);
        int code = 3;   // always the Z-axis of the plane (com1) that is the separating axis

        std::vector<DetectedCollision> collisions;
        BoxBoxCollider::generateContactsForFaceSomethingCollision(
            &plane->com(), hsplane, &box->com(), hsbox,
            plane->normal(), code, collisions
        );

        auto rd = std::random_device {}; 
        auto rng = std::default_random_engine { rd() };
        std::shuffle(std::begin(collisions), std::end(collisions), rng);

        scene->_new_collisions.insert(scene->_new_collisions.end(), collisions.begin(), collisions.end());

        // naively check all the vertices
        // auto test_vertex = [&](const Vec3r& dirs)
        // {
        //     const Vec3r vert = box->com().position + 
        //         dirs[0] * box->com().orientation.col(0) * hsbox[0] +
        //         dirs[1] * box->com().orientation.col(1) * hsbox[1] +
        //         dirs[2] * box->com().orientation.col(2) * hsbox[2];
            
        //     Real vert_proj = vert.dot(pn);
        //     if (vert_proj <= plane_proj)
        //     {
        //         RigidRigidCollision new_collision;
        //         new_collision.normal = plane->normal();
        //         new_collision.particle1 = &plane->com();
        //         new_collision.cp_local1 = Vec3r::Zero();
        //         new_collision.particle2 = &box->com();
        //         new_collision.cp_local2 = box->com().orientation.transpose() * (vert - box->com().position);

        //         scene->_new_collisions.push_back(std::move(new_collision));
        //     }
        // };

        // test_vertex(Vec3r(1,1,1));
        // test_vertex(Vec3r(1,-1,-1));
        // test_vertex(Vec3r(1,1,-1));
        // test_vertex(Vec3r(1,-1,1));

        // test_vertex(Vec3r(-1,-1,-1));
        // test_vertex(Vec3r(-1, 1, 1));
        // test_vertex(Vec3r(-1,-1,1));
        // test_vertex(Vec3r(-1,1,-1));
    }

}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRodSegment* segment)
{

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
    // std::cout << "Potential collision between a sphere and rod segment!" << std::endl;

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
    Real combined_radius = sphere->radius() + segment->radius();

    if (sq_dist < combined_radius*combined_radius)
    {
        // Collision!
        // special case: sphere center is almost on the line segment between the two rod nodes
        // normal should just be any perpendicular vector to the line
        Vec3r normal;
        if (sq_dist < 1e-8)
        {
            Vec3r segment_vec = endpoint2 - endpoint1;
            // get an arbitrary vector thats NOT parallel
            if (segment_vec[0] < segment_vec[1])
                normal = segment_vec.cross(Vec3r(1,0,0));
            else
                normal = segment_vec.cross(Vec3r(0,1,0));
        }
        else
        {
            normal = diff / std::sqrt(sq_dist);
        }

        Vec3r sphere_cp_local = -sphere->com().orientation.transpose() * normal*sphere->radius();

        RigidSegmentCollision new_collision;
        new_collision.alpha = t;
        new_collision.normal = normal;
        new_collision.radius = segment->radius();
        new_collision.segment_particle1 = segment->particle1();
        new_collision.segment_particle2 = segment->particle2();
        new_collision.rb_particle = &sphere->com();
        new_collision.rb_cp_local = sphere_cp_local;
        scene->_new_collisions.push_back(std::move(new_collision));    
    }
    

}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2)
{
    if (box1 == box2)
        return;

    std::vector<DetectedCollision> collisions = BoxBoxCollider::collideBoxes(&box1->com(), box1->size(), &box2->com(), box2->size());
    scene->_new_collisions.insert(scene->_new_collisions.end(), collisions.begin(), collisions.end());
}


void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box, SimObject::XPBDRodSegment* segment)
{
    std::cout << "Potential collision between a box and a rod segment!" << std::endl;
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRodSegment* segment1, SimObject::XPBDRodSegment* segment2)
{
    if (segment1 == segment2)
        return;

    if (segment1->particle1() == segment2->particle2() || segment1->particle2() == segment2->particle1())
        return;

    const Vec3r& p1 = segment1->particle1()->position;
    const Vec3r& p2 = segment1->particle2()->position;
    const Vec3r& p3 = segment2->particle1()->position;
    const Vec3r& p4 = segment2->particle2()->position;
    auto [beta1, beta2] = Math::findClosestPointsOnLineSegments(p1, p2, p3, p4);
    
    // const Vec3r cp_rod1 = segment1->pointOnSegment(beta1);
    // const Vec3r cp_rod2 = segment2->pointOnSegment(beta2);
    const Vec3r cp_rod1 = p1 + beta1*(p2 - p1);
    const Vec3r cp_rod2 = p3 + beta2*(p4 - p3);

    Vec3r diff = cp_rod2 - cp_rod1;
    Real sq_dist = diff.squaredNorm();
    Real rads_sq = (segment1->radius() + segment2->radius()) * (segment1->radius() + segment2->radius());
    if (sq_dist < rads_sq)
    {

        if (segment1->size() == 1 && segment2->size() == 1)
        {
            Vec3r normal;
            if (sq_dist < 1e-6)
            {
                normal = (p2 - p1).cross(p4 - p3);
                normal = normal.normalized();
            }
            else
            {
                normal = diff / std::sqrt(sq_dist);
            }
            
            Collision::SegmentSegmentCollision new_collision;
            new_collision.alpha1 = beta1;
            new_collision.alpha2 = beta2;
            new_collision.radius1 = segment1->radius();
            new_collision.radius2 = segment2->radius();
            new_collision.normal = normal;
            new_collision.segment1_particle1 = segment1->particle1();
            new_collision.segment1_particle2 = segment1->particle2();
            new_collision.segment2_particle1 = segment2->particle1();
            new_collision.segment2_particle2 = segment2->particle2();
            scene->_new_collisions.push_back(std::move(new_collision));
        }

        else
        {
            // collision between two collision segments
            // need to add collisions between each of the individual length=1 segments in both segments
            SimObject::XPBDRod* rod1 = segment1->rod();
            SimObject::XPBDRod* rod2 = segment2->rod();
            for (int ind1 = segment1->index1(); ind1 < segment1->index2(); ind1++)
            {
                for (int ind2 = segment2->index1(); ind2 < segment2->index2(); ind2++)
                {
                    SimObject::XPBDRodSegment subsegment1(rod1, ind1, ind1+1);
                    SimObject::XPBDRodSegment subsegment2(rod2, ind2, ind2+1);
                    _checkCollision(scene, &subsegment1, &subsegment2);
                }
            }
        }

        

        

    }

    // std::cout << "Potential collision between two rod segments!" << std::endl;
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