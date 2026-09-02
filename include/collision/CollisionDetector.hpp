#pragma once

#include "common/common.hpp"
#include "config/CollisionSceneConfig.hpp"
#include "collision/CollisionObject.hpp"
#include "collision/LBVH.hpp"
#include "collision/CollisionTypes.hpp"
#include "collision/sdf/SDF.hpp"

#include "simobject/OrientedParticle.hpp"
#include "simobject/rod/XPBDHigherOrderRod.hpp"

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

struct CollisionDetector
{
private:
    /** Collision objects being collided */
    std::vector<CollisionObject> _collision_objects;

    /** BVH */
    LBVH _lbvh;

    /** Cache for potential collisions. */
    std::vector<std::pair<unsigned, unsigned>> _potential_collisions;

    /** Current detected collisions and accompanying order sorted by key */
    std::vector<DetectedCollision> _detected_collisions;

    /** Keep track of particles that have joints defined between them.
     * We want to skip collisions between these pairs of particles.
     * Pairs are stored in increasing address value.
     */
    std::unordered_set<std::pair<const SimObject::OrientedParticle*, const SimObject::OrientedParticle*>, ParticlePairHash> _joint_pairs;

    /** Whether or not rod-rod collisions are enabled */
    bool _rod_rod_collisions = true;

    /** Static dispatch table for colliding pairs of objects during narrow-phase collision detection */
    using CollisionFunc = void(*)(CollisionDetector* cd, void*, void*);
    static CollisionFunc _collision_table[static_cast<int>(ColliderType::COUNT)][static_cast<int>(ColliderType::COUNT)];

    /** Whether or not the collision table has been initialized yet */
    static bool _collision_table_initialized;

    /** Perform narrow-phase collision detection. */
    inline void _narrowPhaseCollisionDetection();

    /** General triangle-SDF continuous collision detection
     * Follows the implementation described by Pelletier-Guenette et al (2025): https://dl.acm.org/doi/full/10.1145/3747862
     * @param ctx simulation context
     * @param triangle primitive index for the triangle
     * @param rb primitive index for the rigid body
     * @param dt the time step size of the sim
     * @param normal (output) the collision normal
     * @param cp_barys (output) the barycentric coordinates of the contact point on the triangle
     * @param cp_rb_local (output) the contact point on the rigid body, expressed in the local rigid body frame
     * @returns whether or not a collision was detected over the interval [0, dt] given the current velocities
     */
    // inline bool _triangleSDF_CCD(Sim::SimulationContext& ctx, unsigned triangle, unsigned rb, Real dt, Vec3r& normal, Vec3r& cp_barys, Vec3r& cp_rb_local);

    bool _checkJoint(const SimObject::OrientedParticle* p1, const SimObject::OrientedParticle* p2) const;

    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane1, SimObject::XPBDPlane* plane2);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane, SimObject::XPBDRigidSphere* sphere);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane, SimObject::XPBDRigidBox* box);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane, SimObject::RodCollisionSegment* segment);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDPlane* plane, SimObject::XPBDRigidMesh* mesh);

    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidSphere* sphere1, SimObject::XPBDRigidSphere* sphere2);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidBox* box);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidSphere* sphere, SimObject::RodCollisionSegment* segment);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidSphere* sphere, SimObject::XPBDRigidMesh* mesh);

    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidBox* box1, SimObject::XPBDRigidBox* box2);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidBox* box, SimObject::RodCollisionSegment* segment);
    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidBox* box, SimObject::XPBDRigidMesh* mesh);
    
    static void _checkCollision(CollisionDetector* cd, SimObject::RodCollisionSegment* segment1, SimObject::RodCollisionSegment* segment2);
    static void _checkCollision(CollisionDetector* cd, SimObject::RodCollisionSegment* segment, SimObject::XPBDRigidMesh* mesh);

    static void _checkCollision(CollisionDetector* cd, SimObject::XPBDRigidMesh* mesh1, SimObject::XPBDRigidMesh* mesh2);

    void _checkRigidSegmentCollision(SimObject::XPBDRigidBody_Base* rb, const SDF* rb_sdf, SimObject::RodCollisionSegment* segment);

    static void _initCollisionTable();

public:
    CollisionDetector() = default;
    CollisionDetector(const Config::CollisionSceneConfig& config);

    /** Adds a object to the spatial hashing cd. */
    template <typename ObjectType>
    void addObject(ObjectType* obj)
    {
        // create collision object
        _collision_objects.emplace_back(obj);
    }

    /** Specific overload for rods - create a collision object for each rod segment. */
    template <typename ElementType>
    void addObject(SimObject::XPBDRod_<ElementType>* rod)
    {
        // create a collision object for each rod segment
        std::vector<SimObject::RodCollisionSegment>& rod_segments = rod->collisionSegments();
        for (auto& segment : rod_segments)
        {
            _collision_objects.emplace_back(&segment);
        }
    }

    void addObject(SimObject::XPBDCubicHermiteRod* rod)
    {
        addObject( (SimObject::XPBDRod_<SimObject::CubicHermiteRodElement>*)rod );
    }

    void addJoint(const SimObject::OrientedParticle* p1, const SimObject::OrientedParticle* p2)
    {
        const SimObject::OrientedParticle* pmin = std::min(p1, p2, std::less<const SimObject::OrientedParticle*>{});
        const SimObject::OrientedParticle* pmax = std::max(p1, p2, std::less<const SimObject::OrientedParticle*>{});
        _joint_pairs.emplace(pmin, pmax);
    }

    /** Performs entire collision detection process: 
     * 
     * - Builds BVH 
     * - uses BVH for broad-phase collision detection
     * - Narrow-phase collision detection on potential collisions
     */
    const std::vector<DetectedCollision>& detectCollisions();
    
};

} // namespace Collision