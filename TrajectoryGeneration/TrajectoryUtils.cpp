#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"

namespace Leph
{
Trajectories TrajectoriesInit()
{
  SplineContainer<SmoothSpline> traj;
  traj.add("is_double_support");
  traj.add("is_left_support_foot");
  traj.add("trunk_pos_x");
  traj.add("trunk_pos_y");
  traj.add("trunk_pos_z");
  traj.add("trunk_axis_x");
  traj.add("trunk_axis_y");
  traj.add("trunk_axis_z");
  traj.add("foot_pos_x");
  traj.add("foot_pos_y");
  traj.add("foot_pos_z");
  traj.add("foot_axis_x");
  traj.add("foot_axis_y");
  traj.add("foot_axis_z");

  return traj;
}

void TrajectoriesTrunkFootPos(double t, const Trajectories& traj, Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis,
                              Eigen::Vector3d& footPos, Eigen::Vector3d& footAxis)
{
  // Compute Cartesian positions
  trunkPos =
      Eigen::Vector3d(traj.get("trunk_pos_x").pos(t), traj.get("trunk_pos_y").pos(t), traj.get("trunk_pos_z").pos(t));
  trunkAxis = Eigen::Vector3d(traj.get("trunk_axis_x").pos(t), traj.get("trunk_axis_y").pos(t),
                              traj.get("trunk_axis_z").pos(t));
  footPos =
      Eigen::Vector3d(traj.get("foot_pos_x").pos(t), traj.get("foot_pos_y").pos(t), traj.get("foot_pos_z").pos(t));
  footAxis =
      Eigen::Vector3d(traj.get("foot_axis_x").pos(t), traj.get("foot_axis_y").pos(t), traj.get("foot_axis_z").pos(t));
}
void TrajectoriesTrunkFootVel(double t, const Trajectories& traj, Eigen::Vector3d& trunkPosVel,
                              Eigen::Vector3d& trunkAxisVel, Eigen::Vector3d& footPosVel, Eigen::Vector3d& footAxisVel)
{
  // Compute Cartesian velocities
  trunkPosVel =
      Eigen::Vector3d(traj.get("trunk_pos_x").vel(t), traj.get("trunk_pos_y").vel(t), traj.get("trunk_pos_z").vel(t));
  trunkAxisVel = Eigen::Vector3d(traj.get("trunk_axis_x").vel(t), traj.get("trunk_axis_y").vel(t),
                                 traj.get("trunk_axis_z").vel(t));
  footPosVel =
      Eigen::Vector3d(traj.get("foot_pos_x").vel(t), traj.get("foot_pos_y").vel(t), traj.get("foot_pos_z").vel(t));
  footAxisVel =
      Eigen::Vector3d(traj.get("foot_axis_x").vel(t), traj.get("foot_axis_y").vel(t), traj.get("foot_axis_z").vel(t));
}
void TrajectoriesTrunkFootAcc(double t, const Trajectories& traj, Eigen::Vector3d& trunkPosAcc,
                              Eigen::Vector3d& trunkAxisAcc, Eigen::Vector3d& footPosAcc, Eigen::Vector3d& footAxisAcc)
{
  // Compute Cartesian accelerations
  trunkPosAcc =
      Eigen::Vector3d(traj.get("trunk_pos_x").acc(t), traj.get("trunk_pos_y").acc(t), traj.get("trunk_pos_z").acc(t));
  trunkAxisAcc = Eigen::Vector3d(traj.get("trunk_axis_x").acc(t), traj.get("trunk_axis_y").acc(t),
                                 traj.get("trunk_axis_z").acc(t));
  footPosAcc =
      Eigen::Vector3d(traj.get("foot_pos_x").acc(t), traj.get("foot_pos_y").acc(t), traj.get("foot_pos_z").acc(t));
  footAxisAcc =
      Eigen::Vector3d(traj.get("foot_axis_x").acc(t), traj.get("foot_axis_y").acc(t), traj.get("foot_axis_z").acc(t));
}
void TrajectoriesSupportFootState(double t, const Trajectories& traj, bool& isDoubleSupport,
                                  HumanoidFixedModel::SupportFoot& supportFoot)
{
  // Compute support foot state
  isDoubleSupport = (traj.get("is_double_support").pos(t) >= 0.5 ? true : false);
  supportFoot = (traj.get("is_left_support_foot").pos(t) >= 0.5 ? HumanoidFixedModel::LeftSupportFoot :
                                                                  HumanoidFixedModel::RightSupportFoot);
}

bool TrajectoriesComputeKinematics(double t, const Trajectories& traj, HumanoidFixedModel& model, Eigen::VectorXd& dq,
                                   Eigen::VectorXd& ddq, double* boundIKDistance)
{
  // Compute Cartesian target
  Eigen::Vector3d trunkPos;
  Eigen::Vector3d trunkAxis;
  Eigen::Vector3d footPos;
  Eigen::Vector3d footAxis;
  Eigen::Vector3d trunkPosVel;
  Eigen::Vector3d trunkAxisVel;
  Eigen::Vector3d footPosVel;
  Eigen::Vector3d footAxisVel;
  Eigen::Vector3d trunkPosAcc;
  Eigen::Vector3d trunkAxisAcc;
  Eigen::Vector3d footPosAcc;
  Eigen::Vector3d footAxisAcc;
  bool isDoubleSupport;
  HumanoidFixedModel::SupportFoot supportFoot;
  TrajectoriesTrunkFootPos(t, traj, trunkPos, trunkAxis, footPos, footAxis);
  TrajectoriesTrunkFootVel(t, traj, trunkPosVel, trunkAxisVel, footPosVel, footAxisVel);
  TrajectoriesTrunkFootAcc(t, traj, trunkPosAcc, trunkAxisAcc, footPosAcc, footAxisAcc);
  TrajectoriesSupportFootState(t, traj, isDoubleSupport, supportFoot);
  // Compute DOF positions
  bool isSuccess = model.trunkFootIK(supportFoot, trunkPos, AxisToMatrix(trunkAxis), footPos, AxisToMatrix(footAxis),
                                     boundIKDistance);
  if (!isSuccess)
  {
    return false;
  }
  // Axis differentiation is converted in proper angular
  // velocity and acceleration
  // Compute DOF velocities
  dq = model.trunkFootIKVel(trunkPosVel, AxisDiffToAngularDiff(trunkAxis, trunkAxisVel), footPosVel,
                            AxisDiffToAngularDiff(footAxis, footAxisVel));
  // Compute DOF accelerations
  ddq = model.trunkFootIKAcc(dq, trunkPosVel, AxisDiffToAngularDiff(trunkAxis, trunkAxisVel), footPosVel,
                             AxisDiffToAngularDiff(footAxis, footAxisVel), trunkPosAcc,
                             AxisDiffToAngularDiff(trunkAxis, trunkAxisAcc), footPosAcc,
                             AxisDiffToAngularDiff(footAxis, footAxisAcc));
  // If available, assign base Pitch/Roll DOFs
  if (traj.exist("base_pitch"))
  {
    model.get().setDOF("base_pitch", traj.get("base_pitch").pos(t));
    dq(model.get().getDOFIndex("base_pitch")) = traj.get("base_pitch").vel(t);
    ddq(model.get().getDOFIndex("base_pitch")) = traj.get("base_pitch").acc(t);
  }
  if (traj.exist("base_roll"))
  {
    model.get().setDOF("base_roll", traj.get("base_roll").pos(t));
    dq(model.get().getDOFIndex("base_roll")) = traj.get("base_roll").vel(t);
    ddq(model.get().getDOFIndex("base_roll")) = traj.get("base_roll").acc(t);
  }

  return true;
}

double DefaultCheckState(const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                         const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                         const Eigen::Vector3d& footAxis)
{
  (void)params;
  (void)t;
  double cost = 0.0;
  if (trunkPos.z() < 0.0)
  {
    cost += 1000.0 - 1000.0 * trunkPos.z();
  }
  if (trunkAxis.norm() >= M_PI / 2.0)
  {
    cost += 1000.0 + 1000.0 * (trunkAxis.norm() - M_PI / 2.0);
  }
  if (fabs(footPos.y()) < 2.0 * 0.037)
  {
    cost += 1000.0 + 1000.0 * (2.0 * 0.037 - fabs(footPos.y()));
  }
  if (footPos.z() < -1e6)
  {
    cost += 1000.0 - 1000.0 * footPos.z();
  }
  if (trunkPos.z() - footPos.z() < 0.20)
  {
    cost += 1000.0 - 1000.0 * (trunkPos.z() - footPos.z() - 0.20);
  }
  if (footAxis.norm() >= M_PI / 2.0)
  {
    cost += 1000.0 + 1000.0 * (footAxis.norm() - M_PI / 2.0);
  }

  return cost;
}

double DefaultCheckDOF(const Eigen::VectorXd& params, double t, const HumanoidModel& model)
{
  (void)params;
  (void)t;
  double cost = 0.0;
  if (fabs(model.getDOF("left_hip_yaw")) > M_PI / 3.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("left_hip_yaw")) - M_PI / 3.0);
  }
  if (fabs(model.getDOF("left_hip_roll")) > M_PI / 3.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("left_hip_roll")) - M_PI / 3.0);
  }
  if (fabs(model.getDOF("left_hip_pitch")) > M_PI / 1.5)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("left_hip_pitch")) - M_PI / 1.5);
  }
  if (fabs(model.getDOF("left_knee")) > 3.0 * M_PI / 2.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("left_knee")) - 3.0 * M_PI / 2.0);
  }
  if (fabs(model.getDOF("left_ankle_pitch")) > M_PI / 2.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("left_ankle_pitch")) - M_PI / 2.0);
  }
  if (fabs(model.getDOF("left_ankle_roll")) > M_PI / 3.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("left_ankle_roll")) - M_PI / 3.0);
  }
  if (fabs(model.getDOF("right_hip_yaw")) > M_PI / 3.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("right_hip_yaw")) - M_PI / 3.0);
  }
  if (fabs(model.getDOF("right_hip_roll")) > M_PI / 3.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("right_hip_roll")) - M_PI / 3.0);
  }
  if (fabs(model.getDOF("right_hip_pitch")) > M_PI / 1.5)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("right_hip_pitch")) - M_PI / 1.5);
  }
  if (fabs(model.getDOF("right_knee")) > 3.0 * M_PI / 2.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("right_knee")) - 3.0 * M_PI / 2.0);
  }
  if (fabs(model.getDOF("right_ankle_pitch")) > M_PI / 2.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("right_ankle_pitch")) - M_PI / 2.0);
  }
  if (fabs(model.getDOF("right_ankle_roll")) > M_PI / 3.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("right_ankle_roll")) - M_PI / 3.0);
  }
  // Pitch/Roll base DOF
  if (fabs(model.getDOF("base_pitch")) > M_PI / 4.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("base_pitch")) - M_PI / 4.0);
  }
  if (fabs(model.getDOF("base_roll")) > M_PI / 4.0)
  {
    cost += 1000.0 + 1000.0 * (fabs(model.getDOF("base_roll")) - M_PI / 4.0);
  }

  return cost;
}

}  // namespace Leph
