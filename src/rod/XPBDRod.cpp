#include "rod/XPBDRod.hpp"

namespace Rod
{

XPBDRod::XPBDRod(int num_nodes, Real length, const Vec3r& p0, const Mat3r& R0,
    const CrossSection* cross_section)
    : _num_nodes(num_nodes), _length(length), _cross_section(cross_section)
{
    _nodes.resize(num_nodes);

    _nodes[0].position = p0;
    _nodes[0].orientation = R0;
    for (int i = 1; i < _num_nodes; i++)
    {
        _nodes[i].position = _nodes[i-1].position + _nodes[i-1].orientation * Vec3r(0,0,_length/_num_nodes);
        _nodes[i].orientation = _nodes[i-1].orientation;
    }
}

} // namespace Rod