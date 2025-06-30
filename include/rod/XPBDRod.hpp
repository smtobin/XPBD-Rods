#ifndef __XPBD_ROD_HPP
#define __XPBD_ROD_HPP

#include "common/common.hpp"

#include "rod/CrossSection.hpp"
#include "solver/BlockThomasSolver.hpp"

#include <memory>

namespace Rod
{

class XPBDRod
{
    public:

    struct Node
    {
        Vec3r position;
        Vec3r velocity;
        Mat3r orientation;
        Vec3r ang_velocity;
    };

    template <typename CrossSectionType_>
    XPBDRod(int num_nodes, Real length, Real density, Real E, Real nu, const Vec3r& p0, const Mat3r& R0, 
        const CrossSectionType_& cross_section)
        : _num_nodes(num_nodes), _length(length), _density(density), _E(E), _nu(nu), _solver(num_nodes)
    {
        // make sure CrossSectionType_ is a type of CrossSection
        static_assert(std::is_base_of_v<CrossSection, CrossSectionType_>);

        // compute shear modulus
        _G = E / (2 * (1+_nu));

        // initialize rod state
        _nodes.resize(num_nodes);

        _nodes[0].position = p0;
        _nodes[0].velocity = Vec3r(0,0,0);
        _nodes[0].orientation = R0;
        _nodes[0].ang_velocity = Vec3r(0,0,0);
        for (int i = 1; i < _num_nodes; i++)
        {
            _nodes[i].position = _nodes[i-1].position + _nodes[i-1].orientation * Vec3r(0,0,_length/(_num_nodes-1));
            _nodes[i].velocity = _nodes[i-1].velocity;
            _nodes[i].orientation = _nodes[i-1].orientation;
            _nodes[i].ang_velocity = _nodes[i-1].ang_velocity;
        }

        _cross_section = std::make_unique<CrossSectionType_>(cross_section);
    }

    const std::vector<Node>& nodes() const { return _nodes; }
    const CrossSection* crossSection() const { return _cross_section.get(); }

    void setup();
    void update(Real dt, Real g_accel);

    private:
    void _inertialUpdate(Real dt, Real g_accel);
    void _computeConstraintVec();
    void _computeConstraintGradients();
    void _computeConstraintGradientBlocks();
    void _positionUpdate();
    void _velocityUpdate(Real dt);
    

    private:
    int _num_nodes;
    Real _length;
    Real _dl;

    // material properties
    Real _density;  // density
    Real _E;    // elastic modulus
    Real _nu;   // Poisson ratio
    Real _G;    // shear modulus

    // mass/inertia properties
    Real _m_total;  // total mass
    Real _m_node;    // mass associated with a single node
    Vec3r _I_rot;
    Vec3r _I_rot_inv;

    // pre-allocated vectors to store constraints and constraint gradients
    VecXr _C_vec;
    MatXr _delC_mat;
    MatXr _LHS_mat;
    VecXr _RHS_vec;
    VecXr _inertia_mat_inv;
    VecXr _alpha;
    VecXr _lambda;
    VecXr _dlam;
    VecXr _dx;

    std::vector<Mat6r> _diag_gradient_blocks;
    std::vector<Mat6r> _off_diag_gradient_blocks;

    std::vector<Mat6r> _diag_CMC_blocks;
    std::vector<Mat6r> _off_diag_CMC_blocks;

    std::unique_ptr<CrossSection> _cross_section;

    std::vector<Node> _nodes;
    std::vector<Node> _prev_nodes;

    Solver::SymmetricBlockThomasSolver<6> _solver;


};

} // namespace Rod

#endif // __XPBD_ROD_HPP