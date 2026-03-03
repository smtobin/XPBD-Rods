#pragma once

#include "common/common.hpp"
#include "collision/SpatialHasher.hpp"

#include "collision/sdf/SDF.hpp"

#include "collision/CollisionTypes.hpp"

#include <unordered_set>

namespace Collision
{
struct ParticlePairHash
{
    std::size_t operator()(
        const std::pair<
            const SimObject::OrientedParticle*,
            const SimObject::OrientedParticle*
        >& p
    ) const noexcept
    {
        std::size_t h1 =
            std::hash<const SimObject::OrientedParticle*>{}(p.first);

        std::size_t h2 =
            std::hash<const SimObject::OrientedParticle*>{}(p.second);

        // Standard hash combine (boost-style)
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
};

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

    template <int Order>
    void addObject(SimObject::XPBDRod_<Order>* rod)
    {
        // do nothing for now
    }

    void addObject(SimObject::XPBDPlane* plane)
    {
        // don't add planes to the spatial hasher
        _planes.push_back(plane);
    }

    void addJoint(const SimObject::OrientedParticle* p1, const SimObject::OrientedParticle* p2)
    {
        const SimObject::OrientedParticle* pmin = std::min(p1, p2, std::less<const SimObject::OrientedParticle*>{});
        const SimObject::OrientedParticle* pmax = std::max(p1, p2, std::less<const SimObject::OrientedParticle*>{});
        _joint_pairs.emplace(pmin, pmax);
    }

    // const XPBDCollisionConstraints_Container& detectCollisions();
    const std::vector<DetectedCollision>& detectCollisions();

private:
    bool _checkJoint(const SimObject::OrientedParticle* p1, const SimObject::OrientedParticle* p2) const;

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

    /** Keep track of particles that have joints defined between them.
     * We want to skip collisions between these pairs of particles.
     * Pairs are stored in increasing address value.
     */
    std::unordered_set<std::pair<const SimObject::OrientedParticle*, const SimObject::OrientedParticle*>, ParticlePairHash> _joint_pairs;

    /** Static dispatch table for colliding pairs of objects during narrow-phase collision detection */
    using CollisionFunc = void(*)(CollisionScene* scene, void*, void*);
    static CollisionFunc _collision_table[static_cast<int>(ColliderType::COUNT)][static_cast<int>(ColliderType::COUNT)];

    /** Whether or not the collision table has been initialized yet */
    static bool _collision_table_initialized;
};

} // namespace Collision