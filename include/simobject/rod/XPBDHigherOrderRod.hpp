#pragma once

#include "common/common.hpp"

#include "config/RodConfig.hpp"
#include "simobject/XPBDObject_Base.hpp"
#include "simobject/rod/RodElement.hpp"

#include "constraint/RodElasticGaussPointConstraint.hpp"

#include "solver/BlockBandedSolver.hpp"

namespace SimObject
{

template<int Order>
class XPBDRod_ : public XPBDObject_Base
{
public:
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

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override;

    Real radius() const { return _radius; }
    const std::vector<SimObject::OrientedParticle>& nodes() const { return _nodes; }

private:
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

    /** Total number of internal constraints currently on the rod. */
    int _num_constraints;

    /** If base and/or tip are fixed */
    bool _base_fixed;
    bool _tip_fixed;

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
    std::vector<RodElement<Order>> _elements;

    /** Nodes of the rod (most current state) */
    std::vector<OrientedParticle> _nodes;

    /** Stores the elastic rod constraints.
     * One elastic rod constraint is defined per each rod segment between two nodes (so there is N-1 elastic constraints).
     * The elastic constraints penalize strain energy in the rod.
     */
    std::vector<Constraint::RodElasticGaussPointConstraint<Order>> _elastic_constraints;

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