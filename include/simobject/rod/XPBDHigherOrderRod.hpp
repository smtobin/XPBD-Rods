#pragma once

#include "common/common.hpp"

#include "config/RodConfig.hpp"
#include "simobject/XPBDObject_Base.hpp"

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

private:
    /** Number of nodes (independent DOF) the rod is discretized into.
     * Note this includes "internal" nodes that are defined in between element endpoints (e.g. in quadratic elements).
     */
    int _num_nodes;

    /** Total length of the rod */
    Real _length;

    /** Cross section properties */
    Real _radius;

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

    /** diagonals of the lambda system matrix (fed into the solver) */
    std::vector<std::vector<Mat6r>> _diagonals;

    /** Nodes of the rod (most current state) */
    std::vector<OrientedParticle> _nodes;

    /** Solves the linear lambda system.
     * The lambda system matrix has a block-banded structure, so we can solve the linear system in O(n) time.
     */
    Solver::SymmetricBlockBandedSolver<OrientedParticle::DOF> _solver;
};

} // namespace SimObject