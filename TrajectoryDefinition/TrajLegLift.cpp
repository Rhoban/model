#include "TrajectoryDefinition/TrajLegLift.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

namespace Leph
{
void TrajLegLift::initializeParameters(TrajectoryParameters& trajParams)
{
  // Total time length
  trajParams.set("time_length", true) = 3.0;
  // Time ratio for control points
  trajParams.set("time_ratio_before", true) = 0.3;
  trajParams.set("time_ratio_swap", true) = 0.5;
  trajParams.set("time_ratio_after", true) = 0.7;

  // Position at swap
  trajParams.set("swap_pos_trunk_pos_x", true) =
      0.7 * trajParams.get("static_double_pos_trunk_pos_x") + 0.3 * trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("swap_pos_trunk_pos_y", true) =
      0.7 * trajParams.get("static_double_pos_trunk_pos_y") + 0.3 * trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("swap_pos_trunk_pos_z", true) =
      0.7 * trajParams.get("static_double_pos_trunk_pos_z") + 0.3 * trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("swap_pos_trunk_axis_x", true) =
      0.7 * trajParams.get("static_double_pos_trunk_axis_x") + 0.3 * trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("swap_pos_trunk_axis_y", true) =
      0.7 * trajParams.get("static_double_pos_trunk_axis_y") + 0.3 * trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("swap_pos_trunk_axis_z", true) =
      0.7 * trajParams.get("static_double_pos_trunk_axis_z") + 0.3 * trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("swap_pos_foot_pos_x", false) = trajParams.get("static_double_pos_foot_pos_x");
  trajParams.set("swap_pos_foot_pos_y", false) = trajParams.get("static_double_pos_foot_pos_y");
  trajParams.set("swap_pos_foot_pos_z", false) = trajParams.get("static_double_pos_foot_pos_z");
  // Velocity at swap
  trajParams.set("swap_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("swap_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("swap_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("swap_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("swap_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("swap_vel_trunk_axis_z", true) = 0.0;
  // Acceleration at swap
  trajParams.set("swap_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("swap_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("swap_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("swap_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("swap_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("swap_acc_trunk_axis_z", true) = 0.0;

  // Position at before
  trajParams.set("before_pos_trunk_pos_x", true) =
      0.75 * trajParams.get("static_double_pos_trunk_pos_x") + 0.25 * trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("before_pos_trunk_pos_y", true) =
      0.75 * trajParams.get("static_double_pos_trunk_pos_y") + 0.25 * trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("before_pos_trunk_pos_z", true) =
      0.75 * trajParams.get("static_double_pos_trunk_pos_z") + 0.25 * trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("before_pos_trunk_axis_x", true) =
      0.75 * trajParams.get("static_double_pos_trunk_axis_x") + 0.25 * trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("before_pos_trunk_axis_y", true) =
      0.75 * trajParams.get("static_double_pos_trunk_axis_y") + 0.25 * trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("before_pos_trunk_axis_z", true) =
      0.75 * trajParams.get("static_double_pos_trunk_axis_z") + 0.25 * trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("before_pos_foot_pos_x", false) = trajParams.get("static_double_pos_foot_pos_x");
  trajParams.set("before_pos_foot_pos_y", false) = trajParams.get("static_double_pos_foot_pos_y");
  trajParams.set("before_pos_foot_pos_z", false) = trajParams.get("static_double_pos_foot_pos_z");
  // Velocity at before
  trajParams.set("before_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("before_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("before_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("before_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("before_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("before_vel_trunk_axis_z", true) = 0.0;
  // Acceleration at before
  trajParams.set("before_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("before_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("before_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("before_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("before_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("before_acc_trunk_axis_z", true) = 0.0;

  // Position at after
  trajParams.set("after_pos_trunk_pos_x", true) =
      0.5 * trajParams.get("static_double_pos_trunk_pos_x") + 0.5 * trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("after_pos_trunk_pos_y", true) =
      0.5 * trajParams.get("static_double_pos_trunk_pos_y") + 0.5 * trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("after_pos_trunk_pos_z", true) =
      0.5 * trajParams.get("static_double_pos_trunk_pos_z") + 0.5 * trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("after_pos_trunk_axis_x", true) =
      0.5 * trajParams.get("static_double_pos_trunk_axis_x") + 0.5 * trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("after_pos_trunk_axis_y", true) =
      0.5 * trajParams.get("static_double_pos_trunk_axis_y") + 0.5 * trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("after_pos_trunk_axis_z", true) =
      0.5 * trajParams.get("static_double_pos_trunk_axis_z") + 0.5 * trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("after_pos_foot_pos_x", true) =
      0.5 * trajParams.get("static_double_pos_foot_pos_x") + 0.5 * trajParams.get("static_single_pos_foot_pos_x");
  trajParams.set("after_pos_foot_pos_y", true) =
      0.5 * trajParams.get("static_double_pos_foot_pos_y") + 0.5 * trajParams.get("static_single_pos_foot_pos_y");
  trajParams.set("after_pos_foot_pos_z", true) =
      0.5 * trajParams.get("static_double_pos_foot_pos_z") + 0.5 * trajParams.get("static_single_pos_foot_pos_z");
  // Velocity at after
  trajParams.set("after_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("after_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("after_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("after_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("after_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("after_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("after_vel_foot_pos_x", true) = 0.0;
  trajParams.set("after_vel_foot_pos_y", true) = 0.0;
  trajParams.set("after_vel_foot_pos_z", true) = 0.0;
  // Acceleration at after
  trajParams.set("after_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("after_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("after_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("after_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("after_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("after_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("after_acc_foot_pos_x", true) = 0.0;
  trajParams.set("after_acc_foot_pos_y", true) = 0.0;
  trajParams.set("after_acc_foot_pos_z", true) = 0.0;
}

TrajectoryGeneration::GenerationFunc TrajLegLift::funcGeneration(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> Trajectories {
    // Retrieve timing parameters
    double endTime = trajParams.get("time_length", params);
    double beforeTime = trajParams.get("time_ratio_before", params) * endTime;
    double swapTime = trajParams.get("time_ratio_swap", params) * endTime;
    double afterTime = trajParams.get("time_ratio_after", params) * endTime;

    // Initialize state splines
    Trajectories traj = TrajectoriesInit();

    // Support phase
    traj.get("is_double_support").addPoint(0.0, 1.0);
    traj.get("is_double_support").addPoint(swapTime, 1.0);
    traj.get("is_double_support").addPoint(swapTime, 0.0);
    traj.get("is_double_support").addPoint(endTime, 0.0);
    // Foot support
    traj.get("is_left_support_foot").addPoint(0.0, 1.0);
    traj.get("is_left_support_foot").addPoint(endTime, 1.0);

    // Starting in static double support pose
    trajParams.trajectoriesAssign(traj, 0.0, "static_double", params);
    // Pre swap time
    trajParams.trajectoriesAssign(traj, beforeTime, "before", params);
    // Support swap
    trajParams.trajectoriesAssign(traj, swapTime, "swap", params);
    // Post swap time
    trajParams.trajectoriesAssign(traj, afterTime, "after", params);
    // Ending in single support pose
    trajParams.trajectoriesAssign(traj, endTime, "static_single", params);

    return traj;
  };
}

TrajectoryGeneration::CheckParamsFunc TrajLegLift::funcCheckParams(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> double {
    // Retrieve timing parameters
    double beforeRatio = trajParams.get("time_ratio_before", params);
    double swapRatio = trajParams.get("time_ratio_swap", params);
    double afterRatio = trajParams.get("time_ratio_after", params);
    double timeLength = trajParams.get("time_length", params);
    // Check time length
    if (timeLength > 10.0)
    {
      return 1000.0 + 1000.0 * (timeLength - 10.0);
    }
    if (timeLength < 0.5)
    {
      return 1000.0 + 1000.0 * (0.5 - timeLength);
    }
    // Check support ratio bound
    if (beforeRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (beforeRatio - 0.1);
    }
    if (beforeRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (beforeRatio - 0.9);
    }
    if (swapRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (swapRatio - 0.1);
    }
    if (swapRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (swapRatio - 0.9);
    }
    if (afterRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (afterRatio - 0.1);
    }
    if (afterRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (afterRatio - 0.9);
    }
    if (beforeRatio + 0.05 > swapRatio)
    {
      return 1000.0 - 1000.0 * (swapRatio - beforeRatio - 0.05);
    }
    if (swapRatio + 0.05 > afterRatio)
    {
      return 1000.0 - 1000.0 * (afterRatio - swapRatio - 0.05);
    }
    return 0.0;
  };
}

TrajectoryGeneration::CheckStateFunc TrajLegLift::funcCheckState(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                       const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                       const Eigen::Vector3d& footAxis) -> double {
    double cost = 0.0;
    // Check that the foot is not colliding
    // the ball before contact time
    double endTime = trajParams.get("time_length", params);
    double staticPos = trajParams.get("static_single_pos_foot_pos_x", params);
    if (t < endTime - 0.02 && footPos.x() > staticPos)
    {
      cost += 1000.0 + 1000.0 * (footPos.x() - staticPos);
    }
    // Forward to default state check
    cost += DefaultCheckState(params, t, trunkPos, trunkAxis, footPos, footAxis);
    return cost;
  };
}

TrajectoryGeneration::CheckDOFFunc TrajLegLift::funcCheckDOF(const TrajectoryParameters& trajParams)
{
  return DefaultCheckDOF;
}

TrajectoryGeneration::ScoreFunc TrajLegLift::funcScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncScore(trajParams);
}

TrajectoryGeneration::EndScoreFunc TrajLegLift::funcEndScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncEndScore(trajParams);
}

TrajectoryGeneration::SaveFunc TrajLegLift::funcSave(const TrajectoryParameters& trajParams)
{
  return DefaultFuncSave(trajParams);
}

}  // namespace Leph
