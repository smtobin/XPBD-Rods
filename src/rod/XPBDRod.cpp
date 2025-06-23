#include "rod/XPBDRod.hpp"

namespace Rod
{


void XPBDRod::update()
{
    _nodes.back().position += Vec3r(0,0,0.001);
}

} // namespace Rod