#pragma once

#include "common/common.hpp"

#include "collision/CollisionBucket.hpp"

#include <unordered_set>

namespace Collision
{

class SpatialHasher
{
public:
    using CollisionPairSet = std::unordered_set<CollisionPair, CollisionPairHash>;

    SpatialHasher(Real grid_size, int num_buckets);

    /** Adds a object to the spatial hashing scene. */
    template <typename ObjectType>
    void addObject(ObjectType* obj)
    {
        // create collision object
        _collision_objects.emplace_back(obj);
    }

    /** Specific overload for rods - create a collision object for each rod segment. */
    void addObject(SimObject::XPBDRod* rod);

    void hashObjects();

    const CollisionPairSet& collisionPairs() const { return _collision_pairs; }

    const std::vector<CollisionObject>& collisionObjects() const { return _collision_objects; }

private:
    /** Helper function to add an object to buckets.
     * @param obj - collision object pointer
     */
    void _addObjectToBuckets(CollisionObject* obj);

    /** Computes the min and max indices for a span [min_coord, max_coord] in real-world space. */
    std::pair<int,int> _cellBounds(Real min_coord, Real max_coord) { return std::make_pair( static_cast<int>(std::floor(min_coord/_grid_size)), static_cast<int>(std::floor(max_coord/_grid_size)) ); }
    
    /** Hash function mapping 3 integers (voxel indices) to bucket index. */
    uint32_t _hash3(int i, int j, int k)
    { 
        uint32_t h = ( (uint32_t(i) * 92837111u) ^ (uint32_t(j) * 689287499u) ^ (uint32_t(k) * 283923481u) );
        // break symmetry in hash
        h ^= h >> 16;
        h *= 0x85ebca6b;
        h ^= h >> 13;
        h *= 0xc2b2ae35;
        h ^= h >> 16;

        return h % uint32_t(_num_buckets);
    }

    /** The side length for the grid */
    Real _grid_size;

    /** The number of collision buckets to be used */
    int _num_buckets;

    /** Stores all collision objects in the scene that will be hashed. */
    std::vector<CollisionObject> _collision_objects;

    /** The collision buckets. Objects will be spatially hashed into these buckets.
     * When multiple objects are in the same bucket, narrow-phase collision detection will be performed for each pair of objects.
     */
    std::vector<CollisionBucket> _buckets;

    /** Current "step" that we are on. This increments every time hashObjects() is called.
     * Each bucket keeps track of the latest step where it was edited; if these don't match, the bucket is cleared. 
     * This avoids having to clear all the buckets every time we want to hash.
     */
    unsigned long _step = 0;

    /** Keep track of buckets with collisions, so that we can easily call narrow-phase collision detection on the potentially intersecting pairs of objects. */
    CollisionPairSet _collision_pairs;
};

} // namespace Collision