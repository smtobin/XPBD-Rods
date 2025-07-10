#ifndef __CROSS_SECTION_HPP
#define __CROSS_SECTION_HPP

#include "common/common.hpp"

namespace Rod
{

/** Represents a generic cross section of the rod.
 */
class CrossSection
{
    public:
    /** Returns a vector of points corresponding to the points of the cross-section in the XY plane. Used for making graphics objects. */
    const std::vector<Vec3r>& crossSectionPoints() const { return _points; }
    /** Second moment of area about x-axis. */
    virtual Real Ix() const = 0;
    /** Second moment of area about y-axis. */
    virtual Real Iy() const = 0;
    /** Polar moment of area. */
    virtual Real Iz() const = 0;
    /** Area of cross-section. */
    virtual Real area() const = 0;

    protected:
    std::vector<Vec3r> _points;
};

/** Represents a circular cross section. */
class CircleCrossSection : public CrossSection
{
    public:
    explicit CircleCrossSection(Real radius, int tubular_resolution=20)
        : CrossSection(), _radius(radius), _tubular_resolution(tubular_resolution)
    {
        _points.resize(_tubular_resolution);
        for (int i = 0; i < _tubular_resolution; i++)
        {
            const Real angle = i * 2 * M_PI / _tubular_resolution;
            _points[i] = Vec3r( _radius * std::cos(angle), _radius * std::sin(angle), 0);
        }
    }

    Real Ix() const override { return M_PI * _radius * _radius * _radius * _radius / 4; }
    Real Iy() const override { return Ix(); }
    Real Iz() const override { return 2*Ix(); }
    
    Real area() const override { return M_PI * _radius * _radius; }

    private:
    Real _radius;
    const int _tubular_resolution;
};

} // namespace Rod

#endif // __CROSS_SECTION_HPP