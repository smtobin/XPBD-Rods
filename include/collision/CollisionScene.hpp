#pragma once

#include "common/common.hpp"
#include "collision/SpatialHasher.hpp"

#include "common/VariadicVectorContainer.hpp"
#include "constraint/RigidBodyCollisionConstraint.hpp"

namespace Collision
{

class CollisionScene
{
public:
    CollisionScene();

    CollisionScene(Real grid_size, int num_buckets);

    template <typename ObjectType>
    void addObject(ObjectType* obj)
    {
        _spatial_hasher.addObject(obj);
    }

    const XPBDCollisionConstraints_Container& detectCollisions();

private:
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere1, SimObject::XPBDRigidSphere* sphere2);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidBox* box);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRodSegment* segment);

    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box, SimObject::XPBDRodSegment* segment);
    
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRodSegment* segment1, SimObject::XPBDRodSegment* segment2);

    static void _initCollisionTable();

    /** Container for the newly added collision constraints. */
    XPBDCollisionConstraints_Container _new_collision_constraints;

    /** Performs broad-phase collision detection. Finds potentially colliding pairs of objects. */
    SpatialHasher _spatial_hasher;

    /** Static dispatch table for colliding pairs of objects during narrow-phase collision detection */
    using CollisionFunc = void(*)(CollisionScene* scene, void*, void*);
    static CollisionFunc _collision_table[3][3];

    /** Whether or not the collision table has been initialized yet */
    static bool _collision_table_initialized;
};

} // namespace Collision