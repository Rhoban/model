#ifndef LEPH_TRAJECTORYUTILS_H
#define LEPH_TRAJECTORYUTILS_H

#include "Spline/SmoothSpline.hpp"
#include "Spline/SplineContainer.hpp"
#include "Model/HumanoidFixedModel.hpp"

namespace Leph
{
/**
 * Simple typedef for trajectories container
 */
typedef SplineContainer<SmoothSpline> Trajectories;

/**
 * Return initialized trajectories for
 * trunk/foot ik cartesian with empty splines
 */
Trajectories TrajectoriesInit();

/**
 * Compute from given spline container
 * trajectory Cartesian trunk and foot
 * position/velocity/acceleration
 * and assign it to given vector
 */
void TrajectoriesTrunkFootPos(double t, const Trajectories& traj, Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis,
                              Eigen::Vector3d& footPos, Eigen::Vector3d& footAxis);
void TrajectoriesTrunkFootVel(double t, const Trajectories& traj, Eigen::Vector3d& trunkPosVel,
                              Eigen::Vector3d& trunkAxisVel, Eigen::Vector3d& footPosVel, Eigen::Vector3d& footAxisVel);
void TrajectoriesTrunkFootAcc(double t, const Trajectories& traj, Eigen::Vector3d& trunkPosAcc,
                              Eigen::Vector3d& trunkAxisAcc, Eigen::Vector3d& footPosAcc, Eigen::Vector3d& footAxisAcc);
void TrajectoriesSupportFootState(double t, const Trajectories& traj, bool& isDoubleSupport,
                                  HumanoidFixedModel::SupportFoot& supportFoot);

/**
 * Compute from given Trajectory spline container
 * at given time t the kinematics.
 * The DOF positions with inverse kinematics is computed
 * and assign to the given model.
 * If available, base DOF are also assign.
 * The DOF velocities and accelerations are assign
 * to given vector dq and ddq.
 * False is returned if inverse kinematics fails.
 * If not null, boundIKDistance is a signed "distance"
 * from kinematics bound. If positive, the IK is valid.
 * If negative, the IK is out of bounds.
 */
bool TrajectoriesComputeKinematics(double t, const Trajectories& traj, HumanoidFixedModel& model, Eigen::VectorXd& dq,
                                   Eigen::VectorXd& ddq, double* boundIKDistance = nullptr);

/**
 * Default Cartesian state check function.
 * Return positive cost value
 * if given time and Cartesian state are outside
 * standard valid range
 */
double DefaultCheckState(const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                         const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                         const Eigen::Vector3d& footAxis);

/**
 * Default Joint DOF check function.
 * Return positive cost value if
 * time and joint DOF of given Model are outside
 * standard valid range
 */
double DefaultCheckDOF(const Eigen::VectorXd& params, double t, const HumanoidModel& model);

}  // namespace Leph

#endif
