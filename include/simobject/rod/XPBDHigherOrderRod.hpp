#pragma once

#include "common/common.hpp"

#include "config/RodConfig.hpp"
#include "simobject/XPBDObject_Base.hpp"
#include "simobject/rod/RodElement.hpp"
#include "simobject/rod/RodCollisionSegment.hpp"

#include "constraint/RodElasticGaussPointConstraint.hpp"

#include "solver/BlockBandedSolver.hpp"

namespace SimObject
{

template<typename ElementType_>
class XPBDRod_ : public XPBDObject_Base
{
public:
    using ElementType = ElementType_;
    using ElasticConstraintType = Constraint::RodElasticGaussPointConstraint<ElementType>;

    constexpr static int NUM_EN = ElementType::NumNodes;
    constexpr static int NUM_GP = ElementType::NumGP;

    XPBDRod_(const Config::RodConfig& config);

    virtual std::vector<const OrientedParticle*> particles() const override;

    /** Performs necessary setup to prepare the rod for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup() override;

    /** Updates rod positions based on current velocities. */
    virtual void inertialUpdate(Real dt) override;

    /** Steps the rod forward in time by dt. */
    virtual void internalConstraintSolve(Real dt) override;

    /** Computes the new translational and angular velocities of each node. */
    virtual void velocityUpdate(Real dt) override;

    virtual AABB boundingBox() const override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override { return std::vector<ConstraintAndLambda>{}; }

    Real radius() const { return _radius; }
    bool globalSolve() const { return _global_solve; }

    void setFixedBaseConstraint(const Constraint::FixedJointConstraint* new_fixed_base_constraint);
    void setFixedTipConstraint(const Constraint::FixedJointConstraint* new_fixed_tip_constraint);

    const std::vector<OrientedParticle>& nodes() const { return _nodes; }
    std::vector<OrientedParticle>& nodes() { return _nodes; }

    const std::vector<ElementType>& elements() const { return _elements; }

    const XPBDConstraints_Container& internalConstraints() const { return _internal_constraints; }

    const std::vector<RodCollisionSegment>& collisionSegments() const { return _collision_segments; }
    std::vector<RodCollisionSegment>& collisionSegments() { return _collision_segments; }

protected:
    /** Allocates space based on the number of constraints. */
    void _allocateSpace();

    /** Number of elements the rod is discretized into. */
    int _num_elements;

    /** Number of nodes (independent DOF) the rod is discretized into.
     * Note this includes "internal" nodes that are defined in between element endpoints (e.g. in quadratic elements).
     */
    int _num_nodes;

    /** Total length of the rod */
    Real _length;

    /** Rest length of each element */
    Real _element_rest_length;

    /** Cross section properties */
    Real _radius;
    Real _area;
    Real _Ix;
    Real _Iz;

    /** Rest curvature in the rod */
    Vec3r _curvature;

    /** Collision geometries */
    std::vector<RodCollisionSegment> _collision_segments;

    /** Total number of internal constraints currently on the rod. */
    int _num_constraints;

    /** If base and/or tip are fixed */
    bool _base_fixed;
    bool _tip_fixed;

    /** Base/tip fixed constraints.
     * These are either one-sided or two-sided fixed joint constraints
     */
    std::variant<const Constraint::OneSidedFixedJointConstraint*, const Constraint::FixedJointConstraint*> _fixed_base_constraint;
    std::variant<const Constraint::OneSidedFixedJointConstraint*, const Constraint::FixedJointConstraint*> _fixed_tip_constraint;

    /** Whether or not to do a global solve for all the constraints.
     * If false, the constraints will be added to the top-level Gauss-Seidel solver.
     */
    bool _global_solve;

    /** Material properties */
    Real _density;  // density
    Real _E;    // elastic modulus
    Real _nu;   // Poisson ratio
    Real _G;    // shear modulus

    /** Pre-allocated vectors to store constraints and constraint gradients */
    VecXr _C_vec;
    VecXr _alpha;
    MatXr _delC_mat;
    VecXr _RHS_vec;
    VecXr _inertia_mat_inv;
    VecXr _dlam;
    VecXr _dx;


    /** Element objects corresponding to each element in the rod. */
    std::vector<ElementType> _elements;

    /** Nodes of the rod (most current state) */
    std::vector<OrientedParticle> _nodes;

    /** Store the inverse inertias of each node */
    std::vector<Vec6r> _node_inverse_inertias;

    /** Stores the elastic rod constraints.
     * One elastic rod constraint is defined per each rod segment between two nodes (so there is N-1 elastic constraints).
     * The elastic constraints penalize strain energy in the rod.
     */
    // std::vector<Constraint::RodElasticGaussPointConstraint<Order>> _elastic_constraints;
    // std::vector<Constraint::RodElasticConstraint> _elastic_constraints;

    /** A buffer to store the gradients of all the elastic constraints. Avoids needless recomputation of gradients. */
    std::vector<typename ElasticConstraintType::GradientMatType> _gradient_buffer;

    /** diagonals of the lambda system matrix (fed into the solver) */
    std::vector<std::vector<Mat6r>> _diagonals;

    /** Solves the linear lambda system.
     * The lambda system matrix has a block-banded structure, so we can solve the linear system in O(n) time.
     */
    Solver::SymmetricBlockBandedSolver<OrientedParticle::DOF> _solver;

    /** All constraints will be "ordered" based on which nodes they affect.
     *   i.e. constraints that affect node 0 will come before constraints that affect node 1, etc.
     * This creates a block banded structure in the lambda system matrix that we can solve efficiently with a block banded solver.
     * Since the elastic rod constraints don't change throughout the simulation, we only need to recompute the ordering of constraints when a new
     * external constraint (such as an attachment constraint) is added.
     */
    std::vector<XPBDConstraints_ConstPtrVariantType> _ordered_constraints;
};

} // namespace SimObject