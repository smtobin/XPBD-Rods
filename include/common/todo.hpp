#pragma once
/** DONE:
 * - FixedJointConstraint - a 6 DOF constraint that fixes two particles in both position and orientation (with some transform offset)
 * 
 * - Figure out clean way of making some constraints "one-sided", i.e. only involve one particle
 *      - e.g. A RevoluteJointConstraint where one of the joint axes is fixed, so the constraint updates only involve one particle
 * 
 */

/** TODO:
 * 
 * Implement Macklin and Muller's rigid body XPBD to compare
 * 
 * === CONSTRAINTS ===
 * - General math helpers for constraint gradients involving boxplus and boxminus - and other ways of cleaning up and streamlining implementation
 * 
 * - SphericalJointConstraint - a 3 DOF spherical constraint
 * 
 * - PrismaticJointConstraint - a 5 DOF prismatic constraint
 * 
 * - Figure out a clean way of adding joint limits to joint constraints
 * 
 * - Figure out a clean way of specifying a desired position/orientation with joint constraints, with some compliance
 *      - e.g. a RevoluteJointConstraint that has a torsional spring and a desired angle
 * 
 * - PointOnLine w/ orientation constraint - a constraint for maintaining concentricity of tendons through routing holes
 *      - the position of the routing hole should be on the line between adjacent Cosserat rod noes AND the line should be aligned with the hole direction
 * 
 * - Sliding concentric constraints, more specifically "transferring" PointOnLine constraints between adjacent segments
 *      - first: project the point particle onto the line, and see if it is past the endpoints. If it is, try again on an adjacent segment.
 *      - should this happen during the inertial update?
 * 
 * 
 * === SOLVERS ===
 * - Global solver that uses Eigen's sparse matrix solvers
 * 
 * - Banded solver (not block-banded)?
 * 
 * 
 * 
 * === OBJECT GROUPS ===
 * - Dummy parallel continuum robot - 3 Cosserat rods and a rigid platform connecting them
 * 
 */