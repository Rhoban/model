#ifndef LEPH_RBDLCLOSEDLOOP_H
#define LEPH_RBDLCLOSEDLOOP_H

#include <rbdl/rbdl.h>
#include <Eigen/Dense>

namespace Leph
{
/**
 * Compute the Inverse Dynamics of given RBDL
 * model with given body id fixed (position and
 * orientation) in base coordinates.
 * This fixed body creates a kinematics loop.
 *
 * Model position Q, velocity QDot and acceleration QDDot
 * are given. Computed torques for degrees
 * of freedom are returned.
 *
 * If given model has floating base virtual body, the
 * floating base is used for position and orientation
 * computations and the torques are also evaluate
 * and returned for floating base
 * degrees of freedom.
 *
 * The cartesian 6D moment and linear (Nx, Ny, Nz, Fx, Fy, Fz)
 * contact forces (lambda) at fixed body point are returned
 * in fixedBodyForce if not null.
 *
 * If useInfinityNorm is true, underdetermined torques
 * system is solve by minimizing normInfinity() + norm2()
 * with CMA-ES optimization but with big performance cost.
 */
RigidBodyDynamics::Math::VectorNd RBDLClosedLoopInverseDynamics(
    RigidBodyDynamics::Model& model, const RigidBodyDynamics::Math::VectorNd& Q,
    const RigidBodyDynamics::Math::VectorNd& QDot, const RigidBodyDynamics::Math::VectorNd& QDDot,
    unsigned int fixedBodyId, RigidBodyDynamics::Math::VectorNd* fixedBodyForce, bool useInfinityNorm = false);

}  // namespace Leph

#endif
