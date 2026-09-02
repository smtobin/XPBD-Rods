#include "collision/LBVH.hpp"

#include "common/Algorithm.hpp"

#include <bit>
#include <queue>

namespace Collision
{

void LBVH::_printTreeWithInfoImpl(unsigned node, const std::string& prefix, bool is_left) const
{
    std::string node_str;
    if (node >= numPrimitives())
        node_str = "I" + std::to_string(node - numPrimitives());
    else
        node_str = "L" + std::to_string(node);

    std::stringstream morton_code_ss;
    morton_code_ss << "(" << "aabb=[" << min_x[node] << ", " << min_y[node] << ", " << min_z[node]
            << "] to [" << max_x[node] << ", " << max_y[node] << ", " << max_z[node] << "]";
    if (node < numPrimitives())
        morton_code_ss << ", "
            << "code=" << std::bitset<64>(_morton_code[_sorted_order[node]]) << ")";
    else
        morton_code_ss << ")";

    std::cout << prefix
            << (is_left ? "├── " : "└── ")
            << node_str << " " << morton_code_ss.str() << 
            '\n';

    if (leaf_count[node] > 0) return;

    _printTreeWithInfoImpl(left[node],
            prefix + (is_left ? "│   " : "    "),
            true);

    _printTreeWithInfoImpl(right[node],
            prefix + (is_left ? "│   " : "    "),
            false);
}

std::vector<unsigned> LBVH::nodeDepths() const
{
    std::cout << "Parent size: " << parent.size() << std::endl;
    std::vector<unsigned> depth(parent.size(), 0);

    std::queue<unsigned> q;
    q.push(root);

    while (!q.empty())
    {
        unsigned n = q.front();
        q.pop();

        if (left[n] != INVALID)
        {
            depth[left[n]] = depth[n] + 1;
            q.push(left[n]);
        }

        if (right[n] != INVALID)
        {
            depth[right[n]] = depth[n] + 1;
            q.push(right[n]);
        }
    }

    return depth;
}

void LBVH::build(const std::vector<CollisionObject>& collision_objects)
{
    // initial resize so that the BVH has space for all collision objects
    unsigned n_obj = collision_objects.size();
    resize(n_obj);

    // compute AABBs for each collision object
    SimObject::AABB scene_box = SimObject::AABB::empty();
    for (unsigned i = 0; i < n_obj; i++)
    {
        _aabb[i] = collision_objects[i].boundingBox();
        _centroid[i] = _aabb[i].center();

        // expand the scene box
        scene_box.expand(_aabb[i]);
    }

    // Morton code for each collision object
    Vec3r extent = scene_box.max - scene_box.min;
    extent = extent.cwiseMax(1e-6); // guard against degenerate axes
    for (unsigned i = 0; i < n_obj; i++)
    {
        // normalize centroid between [0,1]^3
        Vec3r normalized_centroid = (_centroid[i] - scene_box.min).cwiseQuotient(extent);
        _morton_code[i] = morton3D_64(normalized_centroid);

        // std::cout << "Normalized centroid for primitive " << p_idx << ": " << normalized_centroid.transpose() << std::endl;
        // std::cout << "Morton code for primitive " << p_idx << ": " << col__morton_code[p_idx] << std::endl;
    }


    /** Sort collision objects by Morton code */
    // use O(n) radixSort
    Algorithm::radixSort(_morton_code, _sorted_order, n_obj);


    /** Construct tree */
    int n = static_cast<int>(n_obj);
    parent.assign(parent.size(), INVALID);
    left.assign(left.size(), INVALID);
    right.assign(right.size(), INVALID);

    auto delta = [&](int i, int j)
    {
        if (j < 0 || j >= n)
            return -1;

        return commonPrefixLen(
            _morton_code[_sorted_order[i]],
            _sorted_order[i], 
            _morton_code[_sorted_order[j]],
            _sorted_order[j]
        );
    };

    // iterate over internal nodes
    for (int i = 0; i < n-1; i++)
    {
        // "direction" of interval
        int d = (delta(i, i + 1) - delta(i, i - 1) >= 0) ? 1 : -1;
        
        // lower bound on LCP length for siblings 
        int dmin = delta(i, i-d);

        // find upper bound on the range length
        int l_max = 128;
        while (delta(i, i+l_max*d) > dmin)
        {
            l_max *= 4;
        }

        // binary search for the exact far end
        int l = 0;
        for (unsigned t = l_max; t >= 1; t/=2)
        {
            if (delta(i, i + (l+t)*d) > dmin)
                l += t;
        }

        int j = i + l*d;   // range end

        // binary search for the split position within [i, j]
        int dnode = delta(i, j);
        int s = 0;
        int div = 2;
        for (int t = (l + div - 1) / div; t >= 1; t = (t == 1 ? 0 : (t + div - 1) / div))
        {
            int new_s = s + t;
            if (new_s < l && delta(i, i + new_s * d) > dnode)
                s = new_s;
        }
        int gamma = i + s * d + std::min(d, 0);


        // index of the current internal node in the global LBVH arrays
        unsigned global_idx = static_cast<unsigned>(i) + n;

        if (std::min(i, j) == gamma)
        {
            left[global_idx] = static_cast<unsigned>(gamma);
            parent[gamma] = global_idx;
        }
        else
        {
            left[global_idx] = static_cast<unsigned>(n + gamma);
            parent[n+gamma] = global_idx;
        }

        if (std::max(i, j) == gamma + 1)
        {
            right[global_idx] = static_cast<unsigned>(gamma + 1);
            parent[gamma + 1] = global_idx;
        }
        else
        {
            right[global_idx] = static_cast<unsigned>(n + gamma + 1);
            parent[n + gamma + 1] = global_idx;
        }
    }

    // find root - start at an arbitrary leaf node and go upwards
    unsigned node = 0;
    while (parent[node] != INVALID)
        node = parent[node];
    root = node;

    // process leaves
    for (int l_idx = 0; l_idx < n; l_idx++)
    {
        leaf_start[l_idx] = l_idx;
        leaf_count[l_idx] = 1;
        subtree_size[l_idx] = 1;
    }

    // internal nodes
    for (int idx = 0; idx+1 < n; idx++)
    {
        leaf_count[n + idx] = 0;
    }



    /** Assemble BVH */
    // refit pass
    std::vector<uint8_t> visited(2 * n - 1);

    // start at leaves, then walk up
    for (unsigned l_idx = 0; l_idx < n; l_idx++)
    {
        SimObject::AABB box = SimObject::AABB::empty();
        for (unsigned i = 0; i < leaf_count[l_idx]; i++)
        {
            unsigned prim = _sorted_order[leaf_start[l_idx] + i];
            box.expand(_aabb[prim]);
        }

        min_x[l_idx] = box.min[0]; min_y[l_idx] = box.min[1]; min_z[l_idx] = box.min[2];
        max_x[l_idx] = box.max[0]; max_y[l_idx] = box.max[1]; max_z[l_idx] = box.max[2];

        unsigned node = parent[l_idx];
        while (node != LBVH::INVALID)
        {
            if (visited[node]++ == 0)
                break; // first thread here - the sibling will finish the union

            // merge the AABB
            unsigned l = left[node];
            unsigned r = right[node];

            min_x[node] = std::min(min_x[l], min_x[r]);
            min_y[node] = std::min(min_y[l], min_y[r]);
            min_z[node] = std::min(min_z[l], min_z[r]);
            max_x[node] = std::max(max_x[l], max_x[r]);
            max_y[node] = std::max(max_y[l], max_y[r]);
            max_z[node] = std::max(max_z[l], max_z[r]);

            subtree_size[node] = subtree_size[l] + subtree_size[r];

            node = parent[node];
        }
        
    }
    
}

void LBVH::traverseSelfIterative(unsigned root, std::vector<std::pair<unsigned, unsigned>>& leaf_pairs)
{
    // stack of node pairs to test against each other
    // (avoids recursion)
    std::vector<std::pair<unsigned,unsigned>> work_stack;
    work_stack.reserve(256);
    work_stack.push_back({root, root});

    // process stack until empty
    while (!work_stack.empty())
    {
        auto [node_a, node_b] = work_stack.back();
        work_stack.pop_back();

        SimObject::AABB box_a{ {min_x[node_a], min_y[node_a], min_z[node_a]},
                    {max_x[node_a], max_y[node_a], max_z[node_a]} };
        SimObject::AABB box_b{ {min_x[node_b], min_y[node_b], min_z[node_b]},
                    {max_x[node_b], max_y[node_b], max_z[node_b]} };

        if (!box_a.overlaps(box_b))
            continue;

        bool leaf_a = leaf_count[node_a] > 0;
        bool leaf_b = leaf_count[node_b] > 0;

        // if both are leaves - add this pair as output for narrow-phase
        if (leaf_a && leaf_b)
        {
            if (node_a < node_b)    // avoid adding both (a,b) and (b,a), and avoid (a,a)
                leaf_pairs.push_back({node_a, node_b});
            continue;
        }

        // if both nodes are the same, we are colliding this subtree against itself
        // add 3 checks: left-left, right-right, right-left
        if (node_a == node_b)
        {
            unsigned l = left[node_a], r = right[node_a];
            work_stack.push_back({l, l});
            work_stack.push_back({r, r});
            work_stack.push_back({l, r});
            continue;
        }

        // traverse pair of subtrees
        // descend down whichever tree is larger (heuristic for balance)
        if (leaf_a || (!leaf_b && subtree_size[node_b] > subtree_size[node_a]))
        {
            work_stack.push_back({node_a, left[node_b]});
            work_stack.push_back({node_a, right[node_b]});
        }
        else
        {
            work_stack.push_back({left[node_a], node_b});
            work_stack.push_back({right[node_a], node_b});
        }
    }
}

} // namespace Collision