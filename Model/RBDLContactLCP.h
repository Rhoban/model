#ifndef LEPH_RBDLCONTACTLCP_H
#define LEPH_RBDLCONTACTLCP_H

#include <rbdl/rbdl.h>
#include <Eigen/Dense>

namespace Leph
{
/**
 * Solve the Linear Complimentary
 * Problem used to compute which contact
 * constraints have to be activated or is going
 * to release.
 * Use Moby-Drake LCP solver.
 * RBDL model, position, velocity and applied
 * torque are given in addition to the constraint
 * set for only considered contact.
 * Inertia offsets (H matrix diagonal) is
 * also given (for joint internal inertia).
 *
 * The constraint set hold unilateral (inequality)
 * for non penetration and bilateral (equality)
 * constraints representing no-slip infinite friction.
 * All bilateral constraints must have non zero value
 * in associated vector isBilateralConstraint.
 * (Associated force lambda will not be constraint by
 * complimentary. Always non zero).
 *
 * See Featherstone's book Chapter 11.3, for
 * details and notations.
 *
 * The computed contact force lambda (with zero
 * and non zero elements) is assigned in force
 * field of constraint set.
 */
void RBDLContactLCP(RigidBodyDynamics::Model& model, const RigidBodyDynamics::Math::VectorNd& Q,
                    const RigidBodyDynamics::Math::VectorNd& QDot, const RigidBodyDynamics::Math::VectorNd& Tau,
                    const RigidBodyDynamics::Math::VectorNd& inertiaOffset, RigidBodyDynamics::ConstraintSet& CS,
                    const Eigen::VectorXi& isBilateralConstraint);

}  // namespace Leph

#endif
