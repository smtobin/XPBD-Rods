#pragma once

#include "common/common.hpp"

#include "simobject/Particle.hpp"

#include "config/RodConfig.hpp"
#include "simobject/rod/RodElement.hpp"
#include "simobject/rod/RodCollisionSegment.hpp"
#include "simobject/rod/XPBDHigherOrderRod.hpp"

#include "constraint/RodElasticGaussPointConstraint.hpp"

#include "solver/BlockBandedSolver.hpp"

namespace SimObject
{

class XPBDCubicHermiteRod : public XPBDRod_<CubicHermiteRodElement>
{
public:
    using ElementType = CubicHermiteRodElement;
    using ElasticConstraintType = XPBDRod_<CubicHermiteRodElement>::ElasticConstraintType;

    constexpr static int NUM_EN = ElementType::NumNodes;
    constexpr static int NUM_GP = ElementType::NumGP;

    XPBDCubicHermiteRod(const Config::RodConfig& config);

    /** Performs necessary setup to prepare the rod for simulation. (sets up constraints, computes mass properties, etc.) */
    virtual void setup() override;

    /** Updates rod positions based on current velocities. */
    virtual void inertialUpdate(Real dt) override;

    /** Steps the rod forward in time by dt. */
    virtual void internalConstraintSolve(Real dt) override;

    /** Computes the new translational and angular velocities of each node. */
    virtual void velocityUpdate(Real dt) override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override { return std::vector<ConstraintAndLambda>{}; }

private:

    /** "Particles" that correspond to the derivative DOF in the Hermite formulation. */
    std::vector<Particle> _dp_DOF;
    std::vector<Particle> _dR_DOF;

    MatXr _inertia_mat_global;
    MatXr _inertia_mat_global_inv;
};

} // namespace SimObject