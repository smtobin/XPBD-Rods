#ifndef __XPBD_ROD_HPP
#define __XPBD_ROD_HPP

#include "common/common.hpp"
#include "common/math.hpp"

#include "config/RodConfig.hpp"
#include "simobject/XPBDObject_Base.hpp"
#include "simobject/rod/CrossSection.hpp"
#include "simobject/OrientedParticle.hpp"
#include "solver/BlockThomasSolver.hpp"
#include "solver/BlockBandedSolver.hpp"
#include "constraint/RodElasticConstraint.hpp"
#include "constraint/FixedJointConstraint.hpp"
#include "constraint/AllConstraints.hpp"

#include <memory>
#include <set>
#include <map>
#include <variant>

namespace SimObject
{

/** A class that computes the dynamics of a discretized Cosserat rod using XPBD.
 * 
 * The XPBD formulation accommodates both position and orientation, and treats orientation carefully as to keep it in SO(3). Rotation matrices are used as the
 * underlying representation for SO(3) orientation.
 * 
 * Each rod has two sets of compliant "constraints": elastic (internal) and everything else (external).
 *   The elastic constraints are formulated to penalize strain energy and are physically derived from the discretized Cosserat strain variables.
 *   Other constraints (like attachment constraints) impose arbitrary external constraints on the nodes of the rod. E.g. for attachment constraints,
 *   one node is fixed (potentially with some compliance) to a reference position and orientation.
 * 
 * The linear subsystem that is solved for the change in lambda is solved directly (i.e. NOT solved iteratively using Gauss-Seidel).
 * This is enabled by ordering the constraints such that the LHS matrix for the lambda system (delC * M^-1 * delC^T + alpha_tilde) is block banded.
 * When this is the case, the system of equations can be solved in O(n) time (without any prefactorization!).
 */
class XPBDRod : public XPBDObject_Base
{
    public:
    // a std::variant type used to store all constraints in order
    using ConstraintConstPtrVariantType = std::variant<const Constraint::RodElasticConstraint*, const Constraint::OneSidedFixedJointConstraint*>;

    template <typename CrossSectionType_>
    XPBDRod(const Config::RodConfig& config, const CrossSectionType_& cross_section)
        : XPBDObject_Base(config),
         _num_nodes(config.nodes()), _length(config.length()), _base_fixed(config.baseFixed()), _tip_fixed(config.tipFixed()),
         _density(config.density()), _E(config.E()), _nu(config.nu()),
         _solver(1, config.nodes())
    {
        // make sure CrossSectionType_ is a type of CrossSection
        static_assert(std::is_base_of_v<CrossSection, CrossSectionType_>);

        // compute shear modulus
        _G = _E / (2 * (1+_nu));

        _cross_section = std::make_unique<CrossSectionType_>(cross_section);

        // compute mass/inertia properties
        _m_total = _length * _cross_section->area() * _density;
        _m_node = _m_total / _num_nodes;
        _I_rot = _m_node / _cross_section->area() * Vec3r(_cross_section->Ix(), _cross_section->Iy(), _cross_section->Iz());
        _I_rot_inv = 1.0/_I_rot.array();

        // initialize rod state
        _nodes.resize(_num_nodes);
        _nodes[0].position = config.initialPosition();
        _nodes[0].lin_velocity = config.initialVelocity();
        _nodes[0].orientation = Math::RotMatFromXYZEulerAngles(config.initialRotation());
        _nodes[0].ang_velocity = config.initialAngularVelocity();
        _nodes[0].mass = _m_node;
        _nodes[0].Ib = _I_rot;
        for (int i = 1; i < _num_nodes; i++)
        {
            _nodes[i].position = _nodes[i-1].position + _nodes[i-1].orientation * Vec3r(0,0,_length/(_num_nodes-1));
            _nodes[i].lin_velocity = _nodes[i-1].lin_velocity;
            _nodes[i].orientation = _nodes[i-1].orientation;
            _nodes[i].ang_velocity = _nodes[i-1].ang_velocity;
            _nodes[i].mass = _m_node;
            _nodes[i].Ib = _I_rot;
        }
    }

    ~XPBDRod() override;
    
    // Explicitly declare move operations
    XPBDRod(XPBDRod&&) noexcept;
    XPBDRod& operator=(XPBDRod&&) noexcept;
    
    // Delete copy operations (can't copy unique_ptr)
    XPBDRod(const XPBDRod&) = delete;
    XPBDRod& operator=(const XPBDRod&) = delete;

    const std::vector<OrientedParticle>& nodes() const { return _nodes; }
    std::vector<OrientedParticle>& nodes() { return _nodes; }

    /** Required override of XPBDObject_Base */
    virtual std::vector<const OrientedParticle*> particles() const override;

    const CrossSection* crossSection() const { return _cross_section.get(); }

    const std::vector<Constraint::RodElasticConstraint>& rodConstraints() const { return _elastic_constraints; }

    /** Performs necessary setup to prepare the rod for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup() override;

    /** Updates rod positions based on current velocities. */
    virtual void inertialUpdate(Real dt) override;

    /** Steps the rod forward in time by dt. */
    virtual void internalConstraintSolve(Real dt) override;

    /** Computes the new translational and angular velocities of each node. */
    virtual void velocityUpdate(Real dt) override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override;

    /** Adds an attachment constraint to the specified node.
     * A pointer to the attachment constraint is returned so that the reference position and orientation can be changed.
     */
    Constraint::OneSidedFixedJointConstraint* addOneSidedFixedJointConstraint(int node_index, const Vec3r& ref_position, const Mat3r& ref_orientation);

    /** Removes an attachment constraint for the specified node. */
    bool removeOneSidedFixedJointConstraint(int node_index, const Constraint::OneSidedFixedJointConstraint* ptr=nullptr);

    private:

    /** Applies whatever positional updates are in the _dx vector to the positions and orientations of each node. */
    void _positionUpdate();

    /** Puts all constraints "in order" into the _ordered_constraints vector.
     * This should be called every time a new constraint is added or removed.
     * The order it puts constraints into is dependent on the nodes that are affected by each constraint,
     *   with constraints affecting node 0 coming before constraints affecting node 1, etc.
     * 
     * This function returns the number of nonzero block diagonals present in the lambda system matrix.
     */
    int _orderConstraints();

    /** Functor that is used in calculating the product delC * M^-1 * delC^T, which arises when calculating the LHS of the lambda system. 
     * Because multiple different types of constraints are present in the _ordered_constraints vector (expressed as a std::variant type),
     *   we must use std::visit and a functor object to compute their inertia-weighted gradient products.
    */
    struct _ConstraintGradientProduct
    {
        _ConstraintGradientProduct(const Vec6r& node_inv_inertia_mat) : _node_inv_inertia_mat(node_inv_inertia_mat) { }

        /** Computes the 6x6 matrix delC1 * M^-1 * delC2^T for two constraints C1 and C2. This forms a block in the block banded lambda system matrix.
         * The constraints C1 and C2 will likely affect different sets of nodes. Thus, we must first find which nodes they both affect,
         * since this is where delC1 * M^-1 * delC2^T will be nonzero.
         * Then, we can compute the resulting matrix product.
         */
        template<typename ConstraintType1, typename ConstraintType2>
        Mat6r operator()(const ConstraintType1* constraint1, const ConstraintType2* constraint2)
        {
            typename ConstraintType1::ParticlePtrArray node_ptrs1 = constraint1->particles();
            typename ConstraintType2::ParticlePtrArray node_ptrs2 = constraint2->particles();

            // find if nodes overlap (assuming that node ptrs are sorted and there are no duplicates)
            unsigned ind1 = 0;
            unsigned ind2 = 0;
            std::vector<OrientedParticle*> overlapping_nodes;
            while (ind1 < node_ptrs1.size() && ind2 < node_ptrs2.size())
            {
                // increment whichever node index is smaller
                if (node_ptrs1[ind1] > node_ptrs2[ind2])
                {
                    ind2++;
                }
                else if (node_ptrs1[ind1] < node_ptrs2[ind2])
                {
                    ind1++;
                }
                // node indices are equal ==> we have overlap!
                else
                {
                    overlapping_nodes.push_back(node_ptrs1[ind1]);
                    ind1++;
                    ind2++;
                }

            }

            Mat6r grad_prod = Mat6r::Zero();
            // if we have no overlap, the product is simply 0
            if (overlapping_nodes.empty())
                return grad_prod;
            
            // add up the matrix product for each of the overlapping indices
            for (const auto& node_ptr : overlapping_nodes)
            {
                grad_prod += constraint1->singleParticleGradient(node_ptr, true) * _node_inv_inertia_mat.asDiagonal() * constraint2->singleParticleGradient(node_ptr, true).transpose();
            }

            return grad_prod;
        }

        const Vec6r& _node_inv_inertia_mat;
    };

    /** Functor that is used in calculating the position updates M^-1 * delC^T * dlam, which is done after we've solved the lamda system. 
     * Because multiple different types of constraints are present in the _ordered_constraints vector (expressed as a std::variant type),
     *   we must use std::visit and a functor object to compute the position updates.
    */
    struct _ComputePositionUpdateForConstraint
    {
        /** Takes in information that we need:
         * dlam: the 6x1 vector corresponding to changes in lambda for this 6x1 constraint
         * dx_ptr: pointer to the position updates vector (i.e. pointer to the _dx class variable)
         * node_inv_inertia_mat: the inverse inertia mat M^-1, represented as a 6x1 vector
         */
        _ComputePositionUpdateForConstraint(const OrientedParticle* first_node_ptr, const Vec6r& dlam, VecXr* dx_ptr, const Vec6r& node_inv_inertia_mat)
            : _first_node_ptr(first_node_ptr), _dlam(dlam), _dx_ptr(dx_ptr), _node_inv_inertia_mat(node_inv_inertia_mat) { }

        /** Computes and adds the constraint's contribution to the overall position updates vector (given dlam and M^-1). */
        template <typename ConstraintType>
        void operator()(const ConstraintType* constraint)
        {
            // go through all the nodes affected by this constraint and add this constraint's contribution to the position update
            typename ConstraintType::ParticlePtrArray node_ptrs = constraint->particles();
            for (const auto& node_ptr : node_ptrs)
            {
                size_t node_index = node_ptr - _first_node_ptr;
                (*_dx_ptr)( Eigen::seqN(6*node_index, 6)) += 
                    _node_inv_inertia_mat.asDiagonal() * constraint->singleParticleGradient(node_ptr, true).transpose() * _dlam;
            }
        }

        const OrientedParticle* _first_node_ptr;
        const Vec6r& _dlam;
        VecXr* _dx_ptr;
        const Vec6r& _node_inv_inertia_mat;
    };
    

    private:
    /** Number of nodes the rod is discretized into. */
    int _num_nodes;
    /** Length of the rod. */
    Real _length;
    /** Rod cross section. */
    std::unique_ptr<CrossSection> _cross_section;
    /** Total number of constraints currently on the rod. */
    int _num_constraints;

    /** If base and/or tip are fixed */
    bool _base_fixed;
    bool _tip_fixed;

    /** Material properties */
    Real _density;  // density
    Real _E;    // elastic modulus
    Real _nu;   // Poisson ratio
    Real _G;    // shear modulus

    /** Mass/inertia properties */
    Real _m_total;  // total mass
    Real _m_node;    // mass associated with a single node
    Vec3r _I_rot;   // rotational inertia
    Vec3r _I_rot_inv;   // inverse rotational inertia

    // pre-allocated vectors to store constraints and constraint gradients
    VecXr _C_vec;
    MatXr _delC_mat;
    VecXr _RHS_vec;
    VecXr _inertia_mat_inv;
    VecXr _dlam;
    VecXr _dx;

    /** diagonals of the lambda system matrix (fed into the solver) */
    std::vector<std::vector<Mat6r>> _diagonals;

    /** Nodes of the rod (most current state) */
    std::vector<OrientedParticle> _nodes;
    /** Nodes of the rod at the end of the last time step (previous state) */
    std::vector<OrientedParticle> _prev_nodes;

    /** Solves the linear lambda system.
     * The lambda system matrix has a block-banded structure, so we can solve the linear system in O(n) time.
     */
    Solver::SymmetricBlockBandedSolver<OrientedParticle::DOF> _solver;
    
    /** Stores the elastic rod constraints.
     * One elastic rod constraint is defined per each rod segment between two nodes (so there is N-1 elastic constraints).
     * The elastic constraints penalize strain energy in the rod.
     */
    std::vector<Constraint::RodElasticConstraint> _elastic_constraints;

    /** Stores attachment constraints.
     * Attachment constraints are external constraints that fix a node (potentially with some compliance) to some reference position and orientation.
     * We store them in a map indexed by node index, thus the attachment constraints are sorted by node index. 
     * Having a sorted container makes it much easier to order all the constraints properly.
     */
    std::multimap<int, Constraint::OneSidedFixedJointConstraint> _attachment_constraints;

    /** All constraints will be "ordered" based on which nodes they affect.
     *   i.e. constraints that affect node 0 will come before constraints that affect node 1, etc.
     * This creates a block banded structure in the lambda system matrix that we can solve efficiently with a block banded solver.
     * Since the elastic rod constraints don't change throughout the simulation, we only need to recompute the ordering of constraints when a new
     * external constraint (such as an attachment constraint) is added.
     */
    std::vector<ConstraintConstPtrVariantType> _ordered_constraints;

};

} // namespace SimObject

#endif // __XPBD_ROD_HPP