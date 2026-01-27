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

    bool addObject(CollisionObject* obj, unsigned long ID)
    {
        if (ID != cur_ID)
        {
            objects.clear();
            cur_ID = ID;
        }

        objects.push_back(obj);

        return objects.size() > 1;
    }
};

} // namespace Collision