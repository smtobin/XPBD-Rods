#ifndef __XPBD_ROD_HPP
#define __XPBD_ROD_HPP

#include "common/common.hpp"

#include "rod/CrossSection.hpp"

namespace Rod
{

class XPBDRod
{
    public:

    struct Node
    {
        Vec3r position;
        Mat3r orientation;
    };

    XPBDRod(int num_nodes, Real length, const Vec3r& p0, const Mat3r& R0, 
        const CrossSection* cross_section);

    const std::vector<Node> nodes() const { return _nodes; }
    const CrossSection* crossSection() const { return _cross_section; }

    private:
    int _num_nodes;
    Real _length;

    const CrossSection* _cross_section;

    std::vector<Node> _nodes;


};

} // namespace Rod

#endif // __XPBD_ROD_HPP