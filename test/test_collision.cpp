#include "common/common.hpp"
#include "collision/CollisionScene.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"

int main()
{
    Collision::CollisionScene();

    Config::XPBDRigidBoxConfig box1_config;
    SimObject::XPBDRigidBox box1(box1_config);
    
    Config::XPBDRigidBoxConfig box2_config;
    SimObject::XPBDRigidBox box2(box2_config);
}