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

const XPBDCollisionConstraints_Container& CollisionScene::detectCollisions()
{
    _new_collision_constraints.clear();

    // run broad-phase collision detection using spatial hashing
    _spatial_hasher.hashObjects();

    // iterate through potentially colliding pairs of objects
    const SpatialHasher::CollisionPairSet& collision_pairs = _spatial_hasher.collisionPairs();
    for (auto& pair : collision_pairs)
    {
        _collision_table[static_cast<int>(pair.obj1->type)][static_cast<int>(pair.obj2->type)](this, pair.obj1->obj, pair.obj2->obj);
    }

    return _new_collision_constraints;
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
        scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(
            &sphere1->com(), r1, 
            &sphere2->com(), r2, 
            collision_normal
        );
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
        scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
            &box->com(), box_closest_point,
            &sphere->com(), cp_sphere_local,
            collision_normal
        );
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

    BoxSDF box1_sdf(box1);

    // get vertices (in local frame) of box 2
    Vec3r v1_local = box2->com().position - box2->size()/2;
    Vec3r v2_local = v1_local; v2_local[0] += box2->size()[0];
    Vec3r v3_local = v2_local; v3_local[1] += box2->size()[1];
    Vec3r v4_local = v1_local; v4_local[1] += box2->size()[1];

    Vec3r v5_local = v1_local; v1_local[2] += box2->size()[2];
    Vec3r v6_local = v5_local; v6_local[0] += box2->size()[0];
    Vec3r v7_local = v6_local; v7_local[1] += box2->size()[1];
    Vec3r v8_local = v5_local; v8_local[1] += box2->size()[1];

    // get vertices (in global frame) of box 2
    Vec3r v1 = box2->com().orientation * v1_local;
    Vec3r v2 = box2->com().orientation * v2_local;
    Vec3r v3 = box2->com().orientation * v3_local;
    Vec3r v4 = box2->com().orientation * v4_local;
    Vec3r v5 = box2->com().orientation * v5_local;
    Vec3r v6 = box2->com().orientation * v6_local;
    Vec3r v7 = box2->com().orientation * v7_local;
    Vec3r v8 = box2->com().orientation * v8_local;

    auto check_vertex = [&](const Vec3r& v)
    {
        Real dist = box1_sdf.evaluate(v);
        if (dist < 0)
        {
            Vec3r grad = box1_sdf.gradient(v);
            Vec3r cp_box1 = v - dist * grad;
            Vec3r cp_box1_local = box1->com().orientation.transpose() * (cp_box1 - box1->com().position);
            Vec3r cp_box2_local = box2->com().orientation.transpose() * (v - box2->com().position);

            scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
                &box1->com(), cp_box1_local,
                &box2->com(), cp_box2_local,
                grad
            );
        }
    };

    auto check_side = [&](const Vec3r& p1, const Vec3r& p2, const Vec3r& p3, const Vec3r& p4, Real max_side_dim)
    {
        // check SDF distance at center
        Vec3r c = (p1 + p2 + p3 + p4) / 4.0;
        if (box1_sdf.evaluate(c) < max_side_dim)
        {
            Vec3r res1 = _frankWolfe(&box1_sdf, p1, p2, p3);
            Vec3r res2 = _frankWolfe(&box1_sdf, p1, p3, p4);

            Real dist1 = box1_sdf.evaluate(res1);
            if (dist1 < 0)
            {
                // res1 is contact point on box2
                // get contact point on box1
                Vec3r grad1 = box1_sdf.gradient(res1);
                Vec3r cp_box1 = res1 - dist1 * grad1;
                Vec3r cp_box1_local = box1->com().orientation.transpose() * (cp_box1 - box1->com().position);
                Vec3r cp_box2_local = box2->com().orientation.transpose() * (res1 - box2->com().position);

                scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
                    &box1->com(), cp_box1_local,
                    &box2->com(), cp_box2_local,
                    grad1
                );

                std::cout << "\n\nBOX-BOX COLLISION!" << std::endl;
                std::cout << "Box1 collision point: " << cp_box1 << std::endl;
                std::cout << "Box2 collision point: " << res1 << std::endl;
                std::cout << "Collision normal: " << grad1 << std::endl;

                // assert(0);
            }

            Real dist2 = box1_sdf.evaluate(res2);
            if (dist2 < 0)
            {
                Vec3r grad2 = box1_sdf.gradient(res2);
                Vec3r cp_box1 = res2 - dist2 * grad2;
                Vec3r cp_box1_local = box1->com().orientation.transpose() * (cp_box1 - box1->com().position);
                Vec3r cp_box2_local = box2->com().orientation.transpose() * (res2 - box2->com().position);

                scene->_new_collision_constraints.template emplace_back<Constraint::RigidBodyCollisionConstraint>(  
                    &box1->com(), cp_box1_local,
                    &box2->com(), cp_box2_local,
                    grad2
                );

                std::cout << "\n\nBOX-BOX COLLISION!" << std::endl;
                std::cout << "Box1 collision point: " << cp_box1 << std::endl;
                std::cout << "Box2 collision point: " << res2 << std::endl;
                std::cout << "Collision normal: " << grad2 << std::endl;

                // assert(0);
            }
        }
    };

    check_side(v1, v2, v3, v4, std::max(box2->size()[0], box2->size()[1]) );
    check_side(v5, v6, v7, v8, std::max(box2->size()[0], box2->size()[1]) );
    check_side(v1, v2, v5, v6, std::max(box2->size()[0], box2->size()[2]) );
    check_side(v3, v4, v7, v8, std::max(box2->size()[0], box2->size()[2]) );
    check_side(v1, v4, v5, v8, std::max(box2->size()[1], box2->size()[2]) );
    check_side(v2, v3, v5, v6, std::max(box2->size()[1], box2->size()[2]) );

    check_vertex(v1);
    check_vertex(v2);
    check_vertex(v3);
    check_vertex(v4);
    check_vertex(v5);
    check_vertex(v6);
    check_vertex(v7);
    check_vertex(v8);

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