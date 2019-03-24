#include <iostream>
#include "TrajectoryDefinition/TrajKickSingleContact.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

namespace Leph
{
void TrajKickSingleContact::initializeParameters(TrajectoryParameters& trajParams)
{
  // Total time length
  trajParams.set("time_length", false) = 1.5;
  // Time ratio for control points
  trajParams.set("time_ratio_retract", true) = 0.3;
  trajParams.set("time_ratio_contact_end", true) = 0.6;
  trajParams.set("time_ratio_recover", true) = 0.7;

  // Kick configuration
  // Contact_start is collision start (foot at x_start)
  // Contact_end is collision end (foot at x_end)
  trajParams.set("kick_x_start", false) = -0.04;
  trajParams.set("kick_x_end", false) = 0.06;
  trajParams.set("kick_y", false) = -0.12;
  trajParams.set("kick_z", false) = 0.06;
  trajParams.set("kick_vel_start", false) = 0.8;
  trajParams.set("kick_vel_end", false) = 1.0;
  trajParams.set("kick_acc", false) =
      splineComputeAcc(trajParams.get("kick_x_start"), trajParams.get("kick_x_end"), trajParams.get("kick_vel_start"),
                       trajParams.get("kick_vel_end"));

  // Position at contact_start
  trajParams.set("contact_start_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("contact_start_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("contact_start_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("contact_start_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("contact_start_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("contact_start_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.cpy("contact_start_pos_foot_pos_x", "kick_x_start");
  trajParams.cpy("contact_start_pos_foot_pos_y", "kick_y");
  trajParams.set("contact_start_pos_foot_pos_z", true) = trajParams.get("kick_z");
  trajParams.set("contact_start_pos_foot_axis_x", false) = 0.0;
  trajParams.set("contact_start_pos_foot_axis_y", false) = 0.0;
  trajParams.set("contact_start_pos_foot_axis_z", false) = 0.0;
  // Velocity at contact_start
  trajParams.set("contact_start_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("contact_start_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("contact_start_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("contact_start_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("contact_start_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("contact_start_vel_trunk_axis_z", true) = 0.0;
  trajParams.cpy("contact_start_vel_foot_pos_x", "kick_vel_start");
  trajParams.set("contact_start_vel_foot_pos_y", false) = 0.0;
  trajParams.set("contact_start_vel_foot_pos_z", true) = 0.0;
  trajParams.set("contact_start_vel_foot_axis_x", false) = 0.0;
  trajParams.set("contact_start_vel_foot_axis_y", false) = 0.0;
  trajParams.set("contact_start_vel_foot_axis_z", false) = 0.0;
  // Acceleration at contact_start
  trajParams.set("contact_start_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("contact_start_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("contact_start_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("contact_start_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("contact_start_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("contact_start_acc_trunk_axis_z", true) = 0.0;
  trajParams.cpy("contact_start_acc_foot_pos_x", "kick_acc");
  trajParams.set("contact_start_acc_foot_pos_y", false) = 0.0;
  trajParams.set("contact_start_acc_foot_pos_z", true) = 0.0;
  trajParams.set("contact_start_acc_foot_axis_x", false) = 0.0;
  trajParams.set("contact_start_acc_foot_axis_y", false) = 0.0;
  trajParams.set("contact_start_acc_foot_axis_z", false) = 0.0;

  // Position at contact_end
  trajParams.set("contact_end_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("contact_end_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("contact_end_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("contact_end_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("contact_end_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("contact_end_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.cpy("contact_end_pos_foot_pos_x", "kick_x_end");
  trajParams.cpy("contact_end_pos_foot_pos_y", "kick_y");
  trajParams.cpy("contact_end_pos_foot_pos_z", "kick_z");
  trajParams.set("contact_end_pos_foot_axis_x", false) = 0.0;
  trajParams.set("contact_end_pos_foot_axis_y", false) = 0.0;
  trajParams.set("contact_end_pos_foot_axis_z", false) = 0.0;
  // Velocity at contact_end
  trajParams.set("contact_end_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("contact_end_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("contact_end_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("contact_end_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("contact_end_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("contact_end_vel_trunk_axis_z", true) = 0.0;
  trajParams.cpy("contact_end_vel_foot_pos_x", "kick_vel_end");
  trajParams.set("contact_end_vel_foot_pos_y", false) = 0.0;
  trajParams.set("contact_end_vel_foot_pos_z", false) = 0.0;
  trajParams.set("contact_end_vel_foot_axis_x", false) = 0.0;
  trajParams.set("contact_end_vel_foot_axis_y", false) = 0.0;
  trajParams.set("contact_end_vel_foot_axis_z", false) = 0.0;
  // Acceleration at contact_end
  trajParams.set("contact_end_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("contact_end_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("contact_end_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("contact_end_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("contact_end_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("contact_end_acc_trunk_axis_z", true) = 0.0;
  trajParams.cpy("contact_end_acc_foot_pos_x", "kick_acc");
  trajParams.set("contact_end_acc_foot_pos_y", false) = 0.0;
  trajParams.set("contact_end_acc_foot_pos_z", true) = 0.0;
  trajParams.set("contact_end_acc_foot_axis_x", false) = 0.0;
  trajParams.set("contact_end_acc_foot_axis_y", false) = 0.0;
  trajParams.set("contact_end_acc_foot_axis_z", false) = 0.0;

  // Position at retract
  trajParams.set("retract_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("retract_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("retract_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("retract_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("retract_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("retract_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("retract_pos_foot_pos_x", true) = trajParams.get("static_single_pos_foot_pos_x");
  trajParams.set("retract_pos_foot_pos_y", true) = trajParams.get("kick_y");
  trajParams.set("retract_pos_foot_pos_z", true) = trajParams.get("static_single_pos_foot_pos_z");
  trajParams.set("retract_pos_foot_axis_x", false) = 0.0;
  trajParams.set("retract_pos_foot_axis_y", false) = 0.0;
  trajParams.set("retract_pos_foot_axis_z", false) = 0.0;
  // Velocity at retract
  trajParams.set("retract_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("retract_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("retract_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("retract_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("retract_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("retract_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("retract_vel_foot_pos_x", false) = 0.0;
  trajParams.set("retract_vel_foot_pos_y", false) = 0.0;
  trajParams.set("retract_vel_foot_pos_z", false) = 0.0;
  trajParams.set("retract_vel_foot_axis_x", false) = 0.0;
  trajParams.set("retract_vel_foot_axis_y", false) = 0.0;
  trajParams.set("retract_vel_foot_axis_z", false) = 0.0;
  // Acceleration at retract
  trajParams.set("retract_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("retract_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("retract_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("retract_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("retract_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("retract_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("retract_acc_foot_pos_x", true) = 0.0;
  trajParams.set("retract_acc_foot_pos_y", false) = 0.0;
  trajParams.set("retract_acc_foot_pos_z", true) = 0.0;
  trajParams.set("retract_acc_foot_axis_x", false) = 0.0;
  trajParams.set("retract_acc_foot_axis_y", false) = 0.0;
  trajParams.set("retract_acc_foot_axis_z", false) = 0.0;

  // Position at recover
  trajParams.set("recover_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("recover_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("recover_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("recover_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("recover_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("recover_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("recover_pos_foot_pos_x", true) = trajParams.get("static_single_pos_foot_pos_x");
  trajParams.cpy("recover_pos_foot_pos_y", "kick_y");
  trajParams.set("recover_pos_foot_pos_z", true) = trajParams.get("static_single_pos_foot_pos_z");
  trajParams.set("recover_pos_foot_axis_x", false) = 0.0;
  trajParams.set("recover_pos_foot_axis_y", false) = 0.0;
  trajParams.set("recover_pos_foot_axis_z", false) = 0.0;
  // Velocity at recover
  trajParams.set("recover_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("recover_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("recover_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("recover_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("recover_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("recover_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("recover_vel_foot_pos_x", true) = 0.0;
  trajParams.set("recover_vel_foot_pos_y", false) = 0.0;
  trajParams.set("recover_vel_foot_pos_z", true) = 0.0;
  trajParams.set("recover_vel_foot_axis_x", false) = 0.0;
  trajParams.set("recover_vel_foot_axis_y", false) = 0.0;
  trajParams.set("recover_vel_foot_axis_z", false) = 0.0;
  // Acceleration at recover
  trajParams.set("recover_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("recover_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("recover_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("recover_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("recover_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("recover_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("recover_acc_foot_pos_x", true) = 0.0;
  trajParams.set("recover_acc_foot_pos_y", false) = 0.0;
  trajParams.set("recover_acc_foot_pos_z", true) = 0.0;
  trajParams.set("recover_acc_foot_axis_x", false) = 0.0;
  trajParams.set("recover_acc_foot_axis_y", false) = 0.0;
  trajParams.set("recover_acc_foot_axis_z", false) = 0.0;
}

TrajectoryGeneration::GenerationFunc TrajKickSingleContact::funcGeneration(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> Trajectories {
    // Retrieve timing parameters
    double endTime = trajParams.get("time_length", params);
    double retractTime = trajParams.get("time_ratio_retract", params) * endTime;
    double contactEndTime = trajParams.get("time_ratio_contact_end", params) * endTime;
    double contactStartTime =
        contactEndTime - splineComputeTime(trajParams.get("kick_x_start", params), trajParams.get("kick_x_end", params),
                                           trajParams.get("kick_vel_start", params),
                                           trajParams.get("kick_vel_end", params));
    double recoverTime = trajParams.get("time_ratio_recover", params) * endTime;

    // Initialize state splines
    Trajectories traj = TrajectoriesInit();

    // Support phase (single support for kick)
    traj.get("is_double_support").addPoint(0.0, 0.0);
    traj.get("is_double_support").addPoint(endTime, 0.0);
    // Support foot
    traj.get("is_left_support_foot").addPoint(0.0, 1.0);
    traj.get("is_left_support_foot").addPoint(endTime, 1.0);

    // Starting in static single support pose
    trajParams.trajectoriesAssign(traj, 0.0, "static_single", params);
    // Pre Kick
    trajParams.trajectoriesAssign(traj, retractTime, "retract", params);
    // Kick begin
    trajParams.trajectoriesAssign(traj, contactStartTime, "contact_start", params);
    // Kick end
    trajParams.trajectoriesAssign(traj, contactEndTime, "contact_end", params);
    // Post Kick
    trajParams.trajectoriesAssign(traj, recoverTime, "recover", params);
    // Ending in single support pose
    trajParams.trajectoriesAssign(traj, endTime, "static_single", params);

    return traj;
  };
}

TrajectoryGeneration::CheckParamsFunc TrajKickSingleContact::funcCheckParams(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> double {
    // Retrieve timing parameters
    double retractRatio = trajParams.get("time_ratio_retract", params);
    double contactEndRatio = trajParams.get("time_ratio_contact_end", params);
    double recoverRatio = trajParams.get("time_ratio_recover", params);
    double timeLength = trajParams.get("time_length", params);
    double contactStartRatio = contactEndRatio - splineComputeTime(trajParams.get("kick_x_start", params),
                                                                   trajParams.get("kick_x_end", params),
                                                                   trajParams.get("kick_vel_start", params),
                                                                   trajParams.get("kick_vel_end", params)) /
                                                     timeLength;
    // Check time length
    if (timeLength > 5.0)
    {
      return 1000.0 + 1000.0 * (timeLength - 5.0);
    }
    if (timeLength < 1.0)
    {
      return 1000.0 + 1000.0 * (1.0 - timeLength);
    }
    // Check ratio bound
    if (retractRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (retractRatio - 0.1);
    }
    if (retractRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (retractRatio - 0.9);
    }
    if (contactStartRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (contactStartRatio - 0.1);
    }
    if (contactStartRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (contactStartRatio - 0.9);
    }
    if (contactEndRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (contactEndRatio - 0.1);
    }
    if (contactEndRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (contactEndRatio - 0.9);
    }
    if (recoverRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (recoverRatio - 0.1);
    }
    if (recoverRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (recoverRatio - 0.9);
    }
    if (retractRatio + 0.05 > contactStartRatio)
    {
      return 1000.0 - 1000.0 * (contactStartRatio - retractRatio - 0.05);
    }
    if (contactStartRatio + 0.05 > contactEndRatio)
    {
      return 1000.0 - 1000.0 * (contactEndRatio - contactStartRatio - 0.05);
    }
    if (contactEndRatio + 0.05 > recoverRatio)
    {
      return 1000.0 - 1000.0 * (recoverRatio - contactEndRatio - 0.05);
    }
    return 0.0;
  };
}

TrajectoryGeneration::CheckStateFunc TrajKickSingleContact::funcCheckState(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                       const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                       const Eigen::Vector3d& footAxis) -> double {
    double cost = 0.0;
    // Forward to default state check
    cost += DefaultCheckState(params, t, trunkPos, trunkAxis, footPos, footAxis);
    return cost;
  };
}

TrajectoryGeneration::CheckDOFFunc TrajKickSingleContact::funcCheckDOF(const TrajectoryParameters& trajParams)
{
  return DefaultCheckDOF;
}

TrajectoryGeneration::ScoreFunc TrajKickSingleContact::funcScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncScore(trajParams);
}

TrajectoryGeneration::EndScoreFunc TrajKickSingleContact::funcEndScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncEndScore(trajParams);
}

TrajectoryGeneration::SaveFunc TrajKickSingleContact::funcSave(const TrajectoryParameters& trajParams)
{
  return DefaultFuncSave(trajParams);
}

double TrajKickSingleContact::splineComputeTime(double pos1, double pos2, double vel1, double vel2)
{
  return 2.0 * (pos2 - pos1) / (vel2 + vel1);
}
double TrajKickSingleContact::splineComputeAcc(double pos1, double pos2, double vel1, double vel2)
{
  double t = splineComputeTime(pos1, pos2, vel1, vel2);
  return (vel2 - vel1) / t;
}

}  // namespace Leph
