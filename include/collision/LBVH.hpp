#pragma once

#include "common/common.hpp"
#include "simobject/AABB.hpp"
#include "collision/CollisionObject.hpp"

#include <vector>
#include <bitset>

namespace Collision
{

/** Stores LBVH nodes in a SoA format.
 * 
 * Node index: [0, n-1] are leaves, [n, 2n-2] are internal nodes
 * 
 * In vanilla LBVH, every leaf is exactly one primitive so leaf_count is always 1. (or 0 for internal nodes)
 * Here we support leaf collapsing - merging small subtrees into a single leaf to reduce tree depth and traversal overhead.
 */
struct LBVH
{
    static constexpr unsigned INVALID = std::numeric_limits<unsigned>::max();
    
    /** Collision geometry, recomputed every frame */
    std::vector<SimObject::AABB> _aabb;
    std::vector<Vec3r> _centroid;
    std::vector<uint64_t> _morton_code;

    /** BVH bookkeeping */
    std::vector<unsigned> _sorted_order;

    /** Fields for the BVH itself */
    std::vector<Real> min_x, min_y, min_z;  // bounding box min coords
    std::vector<Real> max_x, max_y, max_z;  // bounding box max coords
    std::vector<unsigned> left, right;      // left and right children
    std::vector<unsigned> parent;           // parent
    std::vector<unsigned> leaf_start;       // index into sorted order where this leaf's primitives begin
    std::vector<unsigned> leaf_count;       // how many consecutive primitives belong to this leaf
    std::vector<unsigned> subtree_size;     // total leaves under this node (leaves included)
    unsigned root;

    void build(const std::vector<CollisionObject>& collision_objects);

    /** Collides the tree with itself, starting at the designated root, and returning any overlapping leaf nodes.
     * @param root the node index of the subtree root to start at
     * @param leaf_pairs (output) the detected collisions between leaf AABBs. These are pairs of leaf indices, which correspond to the SORTED order in the collision pool.
     */
    void traverseSelfIterative(unsigned root, std::vector<std::pair<unsigned,unsigned>>& leaf_pairs);

    void resize(unsigned num_primitives)
    {
        _aabb.resize(num_primitives);
        _centroid.resize(num_primitives);
        _morton_code.resize(num_primitives);
        _sorted_order.resize(num_primitives);

        unsigned num_nodes = 2 * num_primitives - 1;
        min_x.resize(num_nodes); min_y.resize(num_nodes); min_z.resize(num_nodes);
        max_x.resize(num_nodes); max_y.resize(num_nodes); max_z.resize(num_nodes);
        left.resize(num_nodes);  right.resize(num_nodes);
        parent.resize(num_nodes);
        leaf_start.resize(num_nodes, 0);
        leaf_count.resize(num_nodes, 0);
        subtree_size.resize(num_nodes, 0);
    }

    unsigned numPrimitives() const { return (parent.size() + 1)/2; }

    void printTree() const
    {
        _printTreeImpl(root);
    }

    void printTreeWithInfo() const
    {
        _printTreeWithInfoImpl(root);
    }

    std::vector<unsigned> nodeDepths() const;

private:
    void _printTreeImpl(unsigned node, const std::string& prefix = "", bool is_left = true) const
    {
        std::string node_str;
        if (node >= numPrimitives())
            node_str = "I" + std::to_string(node - numPrimitives());
        else
            node_str = "L" + std::to_string(node);

        std::cout << prefix
                << (is_left ? "├── " : "└── ")
                << node_str << '\n';

        if (leaf_count[node] > 0) return;

        _printTreeImpl(left[node],
                prefix + (is_left ? "│   " : "    "),
                true);

        _printTreeImpl(right[node],
                prefix + (is_left ? "│   " : "    "),
                false);
    }

    void _printTreeWithInfoImpl(unsigned node, const std::string& prefix = "", bool is_left = true) const;

    static inline int countl_zero(uint64_t x)
    {
        return x == 0 ? sizeof(x) * 8 : __builtin_clz(x);
    }

    static inline int commonPrefixLen(uint64_t code_i, unsigned i, uint64_t code_j, unsigned j)
    {
        if (code_i == code_j)
        {
            // ties broken by index — extend LCP count by 64 + clz of index XOR
            return 64 + countl_zero(static_cast<uint64_t>(i) ^ static_cast<uint64_t>(j));
        }
        return countl_zero(code_i ^ code_j);
    }

    /** Morton code generation */
    static inline uint64_t expandBits21(uint64_t v)
    {
        v &= 0x1FFFFFull;
        v = (v | (v << 32)) & 0x1F00000000FFFFull;
        v = (v | (v << 16)) & 0x1F0000FF0000FFull;
        v = (v | (v << 8))  & 0x100F00F00F00F00Full;
        v = (v | (v << 4))  & 0x10C30C30C30C30C3ull;
        v = (v | (v << 2))  & 0x1249249249249249ull;
        return v;
    }

    static inline uint64_t morton3D_64(const Vec3r& p_normalized)
    {
        Real x = std::clamp(p_normalized[0] * 2097152.0, 0.0, 2097151.0); // 2^21
        Real y = std::clamp(p_normalized[1] * 2097152.0, 0.0, 2097151.0);
        Real z = std::clamp(p_normalized[2] * 2097152.0, 0.0, 2097151.0);
        uint64_t xx = expandBits21(static_cast<uint64_t>(x));
        uint64_t yy = expandBits21(static_cast<uint64_t>(y));
        uint64_t zz = expandBits21(static_cast<uint64_t>(z));
        return xx | (yy << 1) | (zz << 2);
    }
};

} // namespace Collision