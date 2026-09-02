#pragma once

#include "common/common.hpp"
#include "collision/CollisionObject.hpp"

#include <vector>

namespace Collision
{

struct CollisionBucket
{
    unsigned long cur_ID = 0;
    std::vector<CollisionObject*> objects;
    std::vector<CollisionObject*> fixed_objects;

    bool addObject(CollisionObject* obj, unsigned long ID)
    {
        if (ID != cur_ID)
        {
            objects.clear();
            cur_ID = ID;
        }

        objects.push_back(obj);

        return (fixed_objects.size() + objects.size()) > 1;
    }

    void addFixedObject(CollisionObject* obj)
    {
        fixed_objects.push_back(obj);
    }
};

} // namespace Collision