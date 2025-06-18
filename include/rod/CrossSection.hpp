#ifndef __CROSS_SECTION_HPP
#define __CROSS_SECTION_HPP

#include "common/common.hpp"

namespace Rod
{

class CrossSection
{
    public:
    const std::vector<Vec3r>& crossSectionPoints() const { return _points; }
    virtual Real Ix() const = 0;
    virtual Real Iy() const = 0;
    virtual Real Iz() const = 0;
    virtual Real area() const = 0;

    protected:
    std::vector<Vec3r> _points;
};

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