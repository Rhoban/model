#include <iostream>
#include "TrajectoryDefinition/TrajStaticPose.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

namespace Leph
{
void TrajStaticPose::initializeParameters(TrajectoryParameters& trajParams)
{
  // Static position
  trajParams.set("static_pose_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("static_pose_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("static_pose_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("static_pose_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("static_pose_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("static_pose_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("static_pose_pos_foot_pos_x", true) = trajParams.get("static_single_pos_foot_pos_x");
  trajParams.set("static_pose_pos_foot_pos_y", true) = trajParams.get("static_single_pos_foot_pos_y");
  trajParams.set("static_pose_pos_foot_pos_z", true) = trajParams.get("static_single_pos_foot_pos_z");
  trajParams.set("static_pose_pos_foot_axis_x", false) = 0.0;
  trajParams.set("static_pose_pos_foot_axis_y", false) = 0.0;
  trajParams.set("static_pose_pos_foot_axis_z", false) = 0.0;
}

TrajectoryGeneration::GenerationFunc TrajStaticPose::funcGeneration(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> Trajectories {
    // Dummy time length
    double endTime = 0.1;

    // Initialize state splines
    Trajectories traj = TrajectoriesInit();

    // Support phase left single support
    traj.get("is_double_support").addPoint(0.0, 0.0);
    traj.get("is_double_support").addPoint(endTime, 0.0);
    // Support foot
    traj.get("is_left_support_foot").addPoint(0.0, 1.0);
    traj.get("is_left_support_foot").addPoint(endTime, 1.0);

    // Start and End in the static single support pose
    trajParams.trajectoriesAssign(traj, 0.0, "static_pose", params);
    trajParams.trajectoriesAssign(traj, endTime, "static_pose", params);

    return traj;
  };
}

TrajectoryGeneration::CheckParamsFunc TrajStaticPose::funcCheckParams(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> double { return 0.0; };
}

TrajectoryGeneration::CheckStateFunc TrajStaticPose::funcCheckState(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                       const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                       const Eigen::Vector3d& footAxis) -> double {
    double cost = 0.0;
    if (footPos.y() > -0.095)
    {
      cost += 1000.0 + 1000.0 * (footPos.y() + 0.095);
    }
    if (trunkPos.z() > 0.285)
    {
      cost += 1000.0 + 1000.0 * (trunkPos.z() - 0.285);
    }
    // Forward to default state check
    cost += DefaultCheckState(params, t, trunkPos, trunkAxis, footPos, footAxis);
    return cost;
  };
}

TrajectoryGeneration::CheckDOFFunc TrajStaticPose::funcCheckDOF(const TrajectoryParameters& trajParams)
{
  return DefaultCheckDOF;
}

TrajectoryGeneration::ScoreFunc TrajStaticPose::funcScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncScore(trajParams);
}

TrajectoryGeneration::EndScoreFunc TrajStaticPose::funcEndScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncEndScore(trajParams);
}

TrajectoryGeneration::SaveFunc TrajStaticPose::funcSave(const TrajectoryParameters& trajParams)
{
  return DefaultFuncSave(trajParams);
}

}  // namespace Leph
