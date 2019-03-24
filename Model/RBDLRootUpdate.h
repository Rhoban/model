#ifndef LEPH_RBDLROOTUPDATE_H
#define LEPH_RBDLROOTUPDATE_H

#include <rbdl/rbdl.h>

namespace Leph
{
/**
 * Re build and return a new RBDL tree
 * model structure where given body id
 * is the new kinematic root
 * A 6 DOF floating base is added if
 * addFloatingBase is true
 */
RigidBodyDynamics::Model RBDLRootUpdate(RigidBodyDynamics::Model& modelOld, size_t newRootBodyId, bool addFloatingBase);

}  // namespace Leph

#endif
