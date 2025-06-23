#ifndef __XPBD_ROD_HPP
#define __XPBD_ROD_HPP

#include "common/common.hpp"

#include "rod/CrossSection.hpp"

#include <memory>

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

    template <typename CrossSectionType_>
    XPBDRod(int num_nodes, Real length, const Vec3r& p0, const Mat3r& R0, 
        const CrossSectionType_& cross_section)
        : _num_nodes(num_nodes), _length(length)
    {
        // make sure CrossSectionType_ is a type of CrossSection
        static_assert(std::is_base_of_v<CrossSection, CrossSectionType_>);

        _nodes.resize(num_nodes);

        _nodes[0].position = p0;
        _nodes[0].orientation = R0;
        for (int i = 1; i < _num_nodes; i++)
        {
            _nodes[i].position = _nodes[i-1].position + _nodes[i-1].orientation * Vec3r(0,0,_length/_num_nodes);
            _nodes[i].orientation = _nodes[i-1].orientation;
        }

        _cross_section = std::make_unique<CrossSectionType_>(cross_section);
    }

    const std::vector<Node> nodes() const { return _nodes; }
    const CrossSection* crossSection() const { return _cross_section.get(); }

    void update();

    private:
    int _num_nodes;
    Real _length;

    std::unique_ptr<CrossSection> _cross_section;

    std::vector<Node> _nodes;


};

} // namespace Rod

#endif // __XPBD_ROD_HPP