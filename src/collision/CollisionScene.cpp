#include "collision/CollisionScene.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rod/XPBDRodSegment.hpp"

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
    : _spatial_hasher(0.5, 1499)
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
}

void CollisionScene::_checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2)
{
    if (box1 == box2)
        return;

    std::cout << "Potential collision between two boxes!" << std::endl;
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

} // namespace Collision