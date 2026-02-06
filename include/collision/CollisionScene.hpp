#pragma once

#include "common/common.hpp"
#include "collision/SpatialHasher.hpp"

#include "collision/sdf/SDF.hpp"

#include "collision/CollisionTypes.hpp"

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

    void addObject(SimObject::XPBDPlane* plane)
    {
        // don't add planes to the spatial hasher
        _planes.push_back(plane);
    }

    // const XPBDCollisionConstraints_Container& detectCollisions();
    const std::vector<DetectedCollision>& detectCollisions();

private:
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane1, SimObject::XPBDPlane* plane2);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRigidSphere* sphere);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRigidBox* box);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDPlane* plane, SimObject::XPBDRodSegment* segment);

    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere1, SimObject::XPBDRigidSphere* sphere2);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidBox* box);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRodSegment* segment);

    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2);
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRigidBox* box, SimObject::XPBDRodSegment* segment);
    
    static void _checkCollision(CollisionScene* scene, SimObject::XPBDRodSegment* segment1, SimObject::XPBDRodSegment* segment2);

    static void _initCollisionTable();

    static Vec3r _frankWolfe(const SDF* sdf, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3);

    /** Store all the newly detected collisions (in the latest call of detectCollisions()).
     * DetectedCollision is a std::variant type that stores different information depending on the type of collision
     * (i.e. Rigid-rigid, rigid-rod segment, rod segment-rod segment, etc.)
     */
    std::vector<DetectedCollision> _new_collisions;

    /** Performs broad-phase collision detection. Finds potentially colliding pairs of objects. */
    SpatialHasher _spatial_hasher;

    /** Store planes (i.e. the ground plane) separately. No need to do spatial hashing, would be a tremendous waste of time. */
    std::vector<SimObject::XPBDPlane*> _planes;

    /** Static dispatch table for colliding pairs of objects during narrow-phase collision detection */
    using CollisionFunc = void(*)(CollisionScene* scene, void*, void*);
    static CollisionFunc _collision_table[static_cast<int>(ColliderType::COUNT)][static_cast<int>(ColliderType::COUNT)];

    /** Whether or not the collision table has been initialized yet */
    static bool _collision_table_initialized;
};

} // namespace Collision