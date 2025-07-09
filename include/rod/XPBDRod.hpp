#ifndef __XPBD_ROD_HPP
#define __XPBD_ROD_HPP

#include "common/common.hpp"

#include "rod/CrossSection.hpp"
#include "rod/XPBDRodNode.hpp"
#include "solver/BlockThomasSolver.hpp"
#include "solver/BlockBandedSolver.hpp"
#include "constraint/RodElasticConstraint.hpp"
#include "constraint/AttachmentConstraint.hpp"

#include <memory>
#include <set>
#include <variant>

namespace Rod
{

class XPBDRod
{
    public:
    using ConstraintConstPtrVariantType = std::variant<const Constraint::RodElasticConstraint*, const Constraint::AttachmentConstraint*>;

    template <typename CrossSectionType_>
    XPBDRod(int num_nodes, Real length, Real density, Real E, Real nu, const Vec3r& p0, const Mat3r& R0, 
        const CrossSectionType_& cross_section)
        : _num_nodes(num_nodes), _length(length), _density(density), _E(E), _nu(nu), _solver(1, num_nodes)
    {
        // make sure CrossSectionType_ is a type of CrossSection
        static_assert(std::is_base_of_v<CrossSection, CrossSectionType_>);

        // compute shear modulus
        _G = E / (2 * (1+_nu));

        // initialize rod state
        _nodes.resize(num_nodes);
        _nodes[0].index = 0;
        _nodes[0].position = p0;
        _nodes[0].velocity = Vec3r(0,0,0);
        _nodes[0].orientation = R0;
        _nodes[0].ang_velocity = Vec3r(0,0,0);
        for (int i = 1; i < _num_nodes; i++)
        {
            _nodes[i].index = i;
            _nodes[i].position = _nodes[i-1].position + _nodes[i-1].orientation * Vec3r(0,0,_length/(_num_nodes-1));
            _nodes[i].velocity = _nodes[i-1].velocity;
            _nodes[i].orientation = _nodes[i-1].orientation;
            _nodes[i].ang_velocity = _nodes[i-1].ang_velocity;
        }

        _cross_section = std::make_unique<CrossSectionType_>(cross_section);
    }

    const std::vector<XPBDRodNode>& nodes() const { return _nodes; }
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

    int _orderConstraints();

    struct ConstraintGradientProduct
    {
        ConstraintGradientProduct(const Vec6r& node_inv_inertia_mat) : _node_inv_inertia_mat(node_inv_inertia_mat) { }

        template<typename ConstraintType1, typename ConstraintType2>
        Mat6r operator()(const ConstraintType1* constraint1, const ConstraintType2* constraint2)
        {
            typename ConstraintType1::NodeIndexArray node_indices1 = constraint1->nodeIndices();
            typename ConstraintType2::NodeIndexArray node_indices2 = constraint2->nodeIndices();

            // find if nodes overlap (assuming that node indices are sorted)
            unsigned ind1 = 0;
            unsigned ind2 = 0;
            std::vector<int> overlapping_indices;
            while (ind1 < node_indices1.size() && ind2 < node_indices2.size())
            {
                if (node_indices1[ind1] > node_indices2[ind2])
                {
                    ind2++;
                }
                else if (node_indices1[ind1] < node_indices2[ind2])
                {
                    ind1++;
                }
                else    // equal!
                {
                    overlapping_indices.push_back(node_indices1[ind1]);
                    ind1++;
                    ind2++;
                }

            }

            Mat6r grad_prod = Mat6r::Zero();
            if (overlapping_indices.empty())
                return grad_prod;
            
            
            for (const auto& node_index : overlapping_indices)
            {
                grad_prod += constraint1->singleNodeGradient(node_index, true) * _node_inv_inertia_mat.asDiagonal() * constraint2->singleNodeGradient(node_index, true).transpose();
            }

            return grad_prod;
        }

        const Vec6r& _node_inv_inertia_mat;
    };

    struct ComputePositionUpdateForConstraint
    {
        ComputePositionUpdateForConstraint(const Vec6r& dlam_ptr, VecXr* dx_ptr, const Vec6r& node_inv_inertia_mat)
            : _dlam_ptr(dlam_ptr), _dx_ptr(dx_ptr), _node_inv_inertia_mat(node_inv_inertia_mat) { }

        template <typename ConstraintType>
        void operator()(const ConstraintType* constraint)
        {
            typename ConstraintType::NodeIndexArray node_indices = constraint->nodeIndices();
            for (const auto& node_index : node_indices)
            {
                (*_dx_ptr)( Eigen::seqN(6*node_index, 6)) += _node_inv_inertia_mat.asDiagonal() * constraint->singleNodeGradient(node_index, true).transpose() * _dlam_ptr;
            }
        }

        const Vec6r& _dlam_ptr;
        VecXr* _dx_ptr;
        const Vec6r& _node_inv_inertia_mat;
    };
    

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
    VecXr _RHS_vec;
    VecXr _inertia_mat_inv;
    VecXr _lambda;
    VecXr _dlam;
    VecXr _dx;

    // diagonals of the lambda system matrix (fed into the solver)
    std::vector<std::vector<Mat6r>> _diagonals;

    std::unique_ptr<CrossSection> _cross_section;

    std::vector<XPBDRodNode> _nodes;
    std::vector<XPBDRodNode> _prev_nodes;

    /** Solves the linear lambda system.
     * The lambda system matrix has a block-banded structure, so we can solve the linear system in O(n) time.
     */
    Solver::SymmetricBlockBandedSolver<6> _solver;
    
    /** Stores the elastic rod constraints.
     * One elastic rod constraint is defined per each rod segment between two nodes (so there is N-1 elastic constraints).
     * The elastic constraints penalize strain energy in the rod.
     */
    std::vector<Constraint::RodElasticConstraint> _elastic_constraints;

    /** Stores attachment constraints.
     * Attachment constraints are external constraints that fix a node (potentially with some compliance) to some reference position and orientation.
     * We store them in a multiset that sorts the attachment constraints by node index. Having a sorted container makes it much easier to order all the constraints properly.
     */
    std::multiset<Constraint::AttachmentConstraint> _attachment_constraints;

    /** All constraints will be "ordered" based on which nodes they affect.
     *   i.e. constraints that affect node 0 will come before constraints that affect node 1, etc.
     * This creates a block banded structure in the lambda system matrix that we can solve efficiently with a block banded solver.
     * Since the elastic rod constraints don't change throughout the simulation, we only need to recompute the ordering of constraints when a new
     * external constraint (such as an attachment constraint) is added.
     */
    std::vector<ConstraintConstPtrVariantType> _ordered_constraints;

};

} // namespace Rod

#endif // __XPBD_ROD_HPP