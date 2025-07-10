#ifndef __XPBD_ROD_NODE_HPP
#define __XPBD_ROD_NODE_HPP

#include "common/common.hpp"

namespace Rod
{

/** Represents a single node of the XPBDRod */
struct XPBDRodNode
{
    constexpr static int NODE_DOF = 6;
    int index;
    Vec3r position;
    Vec3r velocity;
    Mat3r orientation;
    Vec3r ang_velocity;
};

}

#endif // __XPBD_ROD_NODE_HPP