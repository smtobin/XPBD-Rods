#include "collision/SpatialHasher.hpp"

#include "simobject/rod/XPBDRod.hpp"

namespace Collision
{

SpatialHasher::SpatialHasher(Real grid_size, int num_buckets)
    : _grid_size(grid_size), _num_buckets(num_buckets), _buckets(num_buckets)
{
}

void SpatialHasher::addObject(SimObject::XPBDRod* rod)
{
    // create a collision object for each rod segment
    std::vector<SimObject::XPBDRodSegment>& rod_segments = rod->segments();
    for (auto& segment : rod_segments)
    {
        _collision_objects.emplace_back(&segment);
    }
}

void SpatialHasher::hashObjects()
{
    _collision_pairs.clear();
    _step++;
    for (auto& collision_obj : _collision_objects)
    {
        _addObjectToBuckets(&collision_obj);
    }
}

void SpatialHasher::_addObjectToBuckets(CollisionObject* obj)
{
    // std::cout << "\n\n === Object " << obj->obj << std::endl;
    // get bounding box for collision object
    SimObject::AABB bbox = obj->boundingBox();

    // get the voxel index bounds for this object
    auto [min_i, max_i] = _cellBounds(bbox.min[0], bbox.max[0]);
    auto [min_j, max_j] = _cellBounds(bbox.min[1], bbox.max[1]);
    auto [min_k, max_k] = _cellBounds(bbox.min[2], bbox.max[2]);

    // add object to all cells that the object touches
    for (int i = min_i; i <= max_i; i++)
    {
        for (int j = min_j; j <= max_j; j++)
        {
            for (int k = min_k; k <= max_k; k++)
            {
                // get bucket index
                unsigned index = _hash3(i, j, k);
                // std::cout << "  Adding object " << obj->obj << " to bucket " << index << " for (i,j,k): (" << i << ", " << j << ", " << k << ")" << std::endl;
                // add object to bucket
                bool collision = _buckets[index].addObject(obj, _step);

                // if there are other objects add appropriate collision pairs
                if (collision)
                {
                    // std::cout << "   Collision! Adding collision pairs..." << std::endl;
                    // new collision pairs are the ones between the object just added and the already added objects
                    CollisionObject* just_added = _buckets[index].objects.back();
                    for (unsigned ind = 0; ind < _buckets[index].objects.size()-1; ind++)
                    {
                        CollisionObject* already_added = _buckets[index].objects[ind];
                        _collision_pairs.emplace(already_added, just_added);
                        // std::cout << "    (" << just_added->obj << ", " << already_added->obj << ")" << std::endl;
                    }
                        
                }
            }
        }
    }
}

} // namespace Collision