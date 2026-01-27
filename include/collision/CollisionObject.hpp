#pragma once

#include "common/common.hpp"
#include "simobject/AABB.hpp"
#include <cstdint>

namespace Collision
{

enum class ColliderType : uint8_t
{
    Sphere, Box, RodSegment
};

/** An object being tested for collision.
 * The object is stored as a type-tagged pointer, with type according to the ColliderType enum.
 */
struct CollisionObject
{
    void* obj;
    ColliderType type;

    CollisionObject(SimObject::XPBDRigidSphere* sphere)
        : obj(sphere), type(ColliderType::Sphere)
    {}

    CollisionObject(SimObject::XPBDRigidBox* box)
        : obj(box), type(ColliderType::Box)
    {}

    CollisionObject(SimObject::XPBDRodSegment* segment)
        : obj(segment), type(ColliderType::RodSegment)
    {}

    SimObject::AABB boundingBox() const;
};


/** Simple struct for storing two objects that might be in collision, with consistent ordering.
 * The obj1 pointer address is always less than the obj2 pointer address.
 * This makes the pairs unique.
 */
struct CollisionPair
{
    CollisionObject* obj1;
    CollisionObject* obj2;

    CollisionPair(CollisionObject* objA, CollisionObject* objB)
    {
        // obj1 has address < obj2
        // consistent ordering prevents us from storing duplicate collision pairs
        bool objA_is_obj1 = std::less<CollisionObject*>{}(objA, objB);
        if (objA_is_obj1)
        {
            obj1 = objA;
            obj2 = objB;
        }
        else
        {
            obj1 = objB;
            obj2 = objA;
        }
    }

    bool operator==(const CollisionPair& other) const
    {
        return (obj1 == other.obj1) && (obj2 == other.obj2);
    }
};


/** Hash function for CollisionPair
 * Required to store CollisionPairs in a std::unordered_set.
 */
struct CollisionPairHash
{
    size_t operator()(const CollisionPair& cp) const 
    {
        auto h1 = std::hash<void*>{}(cp.obj1);
        auto h2 = std::hash<void*>{}(cp.obj2);
        // Better mixing than simple XOR
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};

} // namespace Collision