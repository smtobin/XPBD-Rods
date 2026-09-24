#pragma once

#include "common/common.hpp"

namespace Collision
{

/** The type of collision geometry
 * 
 * This is used for:
 *  - deciding how to compute bounds in BVH generation
 *  - dispatching the proper narrow-phase collision subroutine
 * 
 * Basically just makes a flat list of supported geometry in the sim, including the types of rigid SDFs supported.
 */
enum class CollisionGeometryType : uint8_t {
    Triangle=0,
    RodSegment,

    // rigid SDF types
    Sphere,
    Box,
    Capsule,

    // total number of collision geometry types
    Count
};

/** Whether or not the collision geometry is an analytic shape.
 * These shapes are represented by analytic signed distance functions (SDFs).
 */
constexpr bool isAnalyticShape(CollisionGeometryType type)
{
    return (
        type == CollisionGeometryType::Sphere ||
        type == CollisionGeometryType::Box ||
        type == CollisionGeometryType::Capsule
    );
}

/** Whether or not the collision geometry has extra information associated with it.
 * 
 */
constexpr bool hasShapeParams(CollisionGeometryType type)
{
    return (
        isAnalyticShape(type) ||
        type == CollisionGeometryType::RodSegment
    );
}

} // namespace Collision