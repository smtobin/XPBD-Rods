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
    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Sphere)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidSphere*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::Box)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRigidBox*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Sphere)][static_cast<int>(ColliderType::RodSegment)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidSphere*>(a), static_cast<SimObject::XPBDRodSegment*>(b));
    };

    // first type is a box
    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Sphere)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::XPBDRigidBox*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::Box)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::XPBDRigidBox*>(b));
    };

    _collision_table[static_cast<int>(ColliderType::Box)][static_cast<int>(ColliderType::RodSegment)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidBox*>(a), static_cast<SimObject::XPBDRodSegment*>(b));
    };

    // first type is a rod segment
    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Sphere)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidSphere*>(b), static_cast<SimObject::XPBDRodSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::Box)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRigidBox*>(b), static_cast<SimObject::XPBDRodSegment*>(a));     // switched
    };

    _collision_table[static_cast<int>(ColliderType::RodSegment)][static_cast<int>(ColliderType::RodSegment)] = [](void* a, void* b) {
        CollisionScene::_checkCollision(static_cast<SimObject::XPBDRodSegment*>(a), static_cast<SimObject::XPBDRodSegment*>(b));
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

void CollisionScene::detectCollisions()
{
    // run broad-phase collision detection using spatial hashing
    _spatial_hasher.hashObjects();

    // iterate through potentially colliding pairs of objects
    const SpatialHasher::CollisionPairSet& collision_pairs = _spatial_hasher.collisionPairs();
    for (auto& pair : collision_pairs)
    {
        _collision_table[static_cast<int>(pair.obj1->type)][static_cast<int>(pair.obj2->type)](pair.obj1->obj, pair.obj2->obj);
    }
}

void CollisionScene::_checkCollision(SimObject::XPBDRigidSphere* sphere1, SimObject::XPBDRigidSphere* sphere2)
{
    if (sphere1 == sphere2)
        return;

    std::cout << "Potential collision between two spheres!" << std::endl;

    Real com_sq_dist = (sphere1->com().position - sphere2->com().position).squaredNorm();
    Real rad_dist = (sphere1->radius() + sphere2->radius())*(sphere1->radius() + sphere2->radius());
    if (com_sq_dist < rad_dist)
        std::cout << "\n\n\n COLLISION BETWEEN SPHERES!\n\n\n" << std::endl;
}

void CollisionScene::_checkCollision(SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidBox* box)
{
    std::cout << "Potential collision between a sphere and a box!" << std::endl;
}

void CollisionScene::_checkCollision(SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRodSegment* segment)
{
    std::cout << "Potential collision between a sphere and rod segment!" << std::endl;
}

void CollisionScene::_checkCollision(SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2)
{
    if (box1 == box2)
        return;

    std::cout << "Potential collision between two boxes!" << std::endl;
}

void CollisionScene::_checkCollision(SimObject::XPBDRigidBox* box, SimObject::XPBDRodSegment* segment)
{
    std::cout << "Potential collision between a box and a rod segment!" << std::endl;
}

void CollisionScene::_checkCollision(SimObject::XPBDRodSegment* segment1, SimObject::XPBDRodSegment* segment2)
{
    if (segment1 == segment2)
        return;

    std::cout << "Potential collision between two rod segments!" << std::endl;
}

} // namespace Collision