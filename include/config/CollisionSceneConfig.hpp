#pragma once

#include "config/Config.hpp"

namespace Config
{
class CollisionSceneConfig : public Config_Base
{
public:
    explicit CollisionSceneConfig()
        : Config_Base()
    {}

    explicit CollisionSceneConfig(const YAML::Node& node)
        : Config_Base(node)
    {
        _extractParameter("hash-voxel-size", node, _hash_voxel_size);
        _extractParameter("num-hash-buckets", node, _num_hash_buckets);
        _extractParameter("rod-rod-collisions", node, _rod_rod_collisions);

    }

    Real hashVoxelSize() const { return _hash_voxel_size.value; }
    int numHashBuckets() const { return _num_hash_buckets.value; }
    bool rodRodCollisions() const { return _rod_rod_collisions.value; }

protected:
    ConfigParameter<Real> _hash_voxel_size = ConfigParameter<Real>(0.5);
    ConfigParameter<int> _num_hash_buckets = ConfigParameter<int>(14909);

    ConfigParameter<bool> _rod_rod_collisions = ConfigParameter<bool>(true);



};

}