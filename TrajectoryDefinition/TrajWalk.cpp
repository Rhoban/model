#include "TrajectoryDefinition/TrajWalk.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

namespace Leph
{
void TrajWalk::initializeParameters(TrajectoryParameters& trajParams)
{
  // Total time length
  trajParams.set("time_length", false) = 0.8;
  // Timing ratio for control points
  trajParams.set("time_ratio_swap", true) = 0.3;
  trajParams.set("time_ratio_apex", true) = 0.7;

  // Foot height rise
  trajParams.set("foot_height", false) = 0.03;
  // Foot lateral distance
  trajParams.set("foot_lateral", true) = trajParams.get("static_double_pos_foot_pos_y");

  // Position at left double support
  trajParams.set("leftds_pos_trunk_pos_x", true) = trajParams.get("static_double_pos_trunk_pos_x");
  trajParams.set("leftds_pos_trunk_pos_y", true) = trajParams.get("static_double_pos_trunk_pos_y") - 0.02;
  trajParams.set("leftds_pos_trunk_pos_z", true) = trajParams.get("static_double_pos_trunk_pos_z");
  trajParams.set("leftds_pos_trunk_axis_x", true) = trajParams.get("static_double_pos_trunk_axis_x");
  trajParams.set("leftds_pos_trunk_axis_y", true) = trajParams.get("static_double_pos_trunk_axis_y");
  trajParams.set("leftds_pos_trunk_axis_z", true) = trajParams.get("static_double_pos_trunk_axis_z");
  trajParams.set("leftds_pos_foot_pos_x", false) = 0.0;
  trajParams.cpy("leftds_pos_foot_pos_y", "foot_lateral");
  trajParams.set("leftds_pos_foot_pos_z", false) = 0.0;
  trajParams.set("leftds_pos_foot_axis_x", false) = 0.0;
  trajParams.set("leftds_pos_foot_axis_y", false) = 0.0;
  trajParams.set("leftds_pos_foot_axis_z", false) = 0.0;
  // Velocity at left double support
  trajParams.set("leftds_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("leftds_vel_trunk_pos_y", true) = 0.2;
  trajParams.set("leftds_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("leftds_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("leftds_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("leftds_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("leftds_vel_foot_pos_x", false) = 0.0;
  trajParams.set("leftds_vel_foot_pos_y", false) = 0.0;
  trajParams.set("leftds_vel_foot_pos_z", false) = 0.0;
  trajParams.set("leftds_vel_foot_axis_x", false) = 0.0;
  trajParams.set("leftds_vel_foot_axis_y", false) = 0.0;
  trajParams.set("leftds_vel_foot_axis_z", false) = 0.0;
  // Acceleration at left double support
  trajParams.set("leftds_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("leftds_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("leftds_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("leftds_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("leftds_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("leftds_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("leftds_acc_foot_pos_x", false) = 0.0;
  trajParams.set("leftds_acc_foot_pos_y", false) = 0.0;
  trajParams.set("leftds_acc_foot_pos_z", false) = 0.0;
  trajParams.set("leftds_acc_foot_axis_x", false) = 0.0;
  trajParams.set("leftds_acc_foot_axis_y", false) = 0.0;
  trajParams.set("leftds_acc_foot_axis_z", false) = 0.0;

  // Position at left single support
  trajParams.set("leftss_pos_trunk_pos_x", true) = trajParams.get("static_double_pos_trunk_pos_x");
  trajParams.set("leftss_pos_trunk_pos_y", true) = trajParams.get("static_double_pos_trunk_pos_y") + 0.04;
  trajParams.set("leftss_pos_trunk_pos_z", true) = trajParams.get("static_double_pos_trunk_pos_z");
  trajParams.set("leftss_pos_trunk_axis_x", true) = trajParams.get("static_double_pos_trunk_axis_x");
  trajParams.set("leftss_pos_trunk_axis_y", true) = trajParams.get("static_double_pos_trunk_axis_y");
  trajParams.set("leftss_pos_trunk_axis_z", true) = trajParams.get("static_double_pos_trunk_axis_z");
  trajParams.set("leftss_pos_foot_pos_x", false) = 0.0;
  trajParams.cpy("leftss_pos_foot_pos_y", "foot_lateral");
  trajParams.set("leftss_pos_foot_pos_z", false) = 0.0;
  trajParams.set("leftss_pos_foot_axis_x", false) = 0.0;
  trajParams.set("leftss_pos_foot_axis_y", false) = 0.0;
  trajParams.set("leftss_pos_foot_axis_z", false) = 0.0;
  // Velocity at left single support
  trajParams.set("leftss_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("leftss_vel_trunk_pos_y", true) = 0.6;
  trajParams.set("leftss_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("leftss_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("leftss_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("leftss_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("leftss_vel_foot_pos_x", false) = 0.0;
  trajParams.set("leftss_vel_foot_pos_y", false) = 0.0;
  trajParams.set("leftss_vel_foot_pos_z", false) = 0.0;
  trajParams.set("leftss_vel_foot_axis_x", false) = 0.0;
  trajParams.set("leftss_vel_foot_axis_y", false) = 0.0;
  trajParams.set("leftss_vel_foot_axis_z", false) = 0.0;
  // Acceleration at left single support
  trajParams.set("leftss_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("leftss_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("leftss_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("leftss_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("leftss_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("leftss_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("leftss_acc_foot_pos_x", false) = 0.0;
  trajParams.set("leftss_acc_foot_pos_y", false) = 0.0;
  trajParams.set("leftss_acc_foot_pos_z", false) = 0.0;
  trajParams.set("leftss_acc_foot_axis_x", false) = 0.0;
  trajParams.set("leftss_acc_foot_axis_y", false) = 0.0;
  trajParams.set("leftss_acc_foot_axis_z", false) = 0.0;

  // Position at left apex
  trajParams.set("leftapex_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("leftapex_pos_trunk_pos_y", true) = -0.02;
  trajParams.set("leftapex_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("leftapex_pos_trunk_axis_x", true) = 0.0;
  trajParams.set("leftapex_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("leftapex_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("leftapex_pos_foot_pos_x", true) = trajParams.get("static_single_pos_foot_pos_x");
  trajParams.set("leftapex_pos_foot_pos_y", true) = trajParams.get("static_single_pos_foot_pos_y");
  trajParams.cpy("leftapex_pos_foot_pos_z", "foot_height");
  trajParams.set("leftapex_pos_foot_axis_x", false) = 0.0;
  trajParams.set("leftapex_pos_foot_axis_y", false) = 0.0;
  trajParams.set("leftapex_pos_foot_axis_z", false) = 0.0;
  // Velocity at left apex
  trajParams.set("leftapex_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("leftapex_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("leftapex_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("leftapex_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("leftapex_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("leftapex_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("leftapex_vel_foot_pos_x", true) = 0.0;
  trajParams.set("leftapex_vel_foot_pos_y", true) = 0.0;
  trajParams.set("leftapex_vel_foot_pos_z", false) = 0.0;
  trajParams.set("leftapex_vel_foot_axis_x", false) = 0.0;
  trajParams.set("leftapex_vel_foot_axis_y", false) = 0.0;
  trajParams.set("leftapex_vel_foot_axis_z", false) = 0.0;
  // Acceleration at left apex
  trajParams.set("leftapex_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("leftapex_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("leftapex_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("leftapex_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("leftapex_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("leftapex_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("leftapex_acc_foot_pos_x", true) = 0.0;
  trajParams.set("leftapex_acc_foot_pos_y", true) = 0.0;
  trajParams.set("leftapex_acc_foot_pos_z", true) = 0.0;
  trajParams.set("leftapex_acc_foot_axis_x", false) = 0.0;
  trajParams.set("leftapex_acc_foot_axis_y", false) = 0.0;
  trajParams.set("leftapex_acc_foot_axis_z", false) = 0.0;
}

TrajectoryGeneration::GenerationFunc TrajWalk::funcGeneration(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> Trajectories {
    // Retrieve timing parameters
    double cycleLength = trajParams.get("time_length", params);
    double swapRatio = trajParams.get("time_ratio_swap", params);
    double apexRatio = trajParams.get("time_ratio_apex", params);
    // Compute timing points
    double leftDoubleSupportTime = 0.0;
    double leftSingleSupportTime = 0.5 * cycleLength * swapRatio;
    double leftApexTime = 0.5 * cycleLength * apexRatio;
    double rightDoubleSupportTime = 0.5 * cycleLength;
    double rightSingleSupportTime = 0.5 * cycleLength * (swapRatio + 1.0);
    double rightApexTime = 0.5 * cycleLength * (apexRatio + 1.0);
    double leftDoubleSupport2Time = cycleLength;

    // Initialize state splines
    Trajectories traj = TrajectoriesInit();

    // Support phase
    traj.get("is_double_support").addPoint(leftDoubleSupportTime, 1.0);
    traj.get("is_double_support").addPoint(leftSingleSupportTime, 1.0);
    traj.get("is_double_support").addPoint(leftSingleSupportTime, 0.0);
    traj.get("is_double_support").addPoint(rightDoubleSupportTime, 0.0);
    traj.get("is_double_support").addPoint(rightDoubleSupportTime, 1.0);
    traj.get("is_double_support").addPoint(rightSingleSupportTime, 1.0);
    traj.get("is_double_support").addPoint(rightSingleSupportTime, 0.0);
    traj.get("is_double_support").addPoint(leftDoubleSupport2Time, 0.0);
    // Foot support
    traj.get("is_left_support_foot").addPoint(leftDoubleSupportTime, 1.0);
    traj.get("is_left_support_foot").addPoint(rightDoubleSupportTime, 1.0);
    traj.get("is_left_support_foot").addPoint(rightDoubleSupportTime, 0.0);
    traj.get("is_left_support_foot").addPoint(leftDoubleSupport2Time, 0.0);

    // Left double support
    trajParams.trajectoriesAssign(traj, leftDoubleSupportTime, "leftds", params);
    // Left single support
    trajParams.trajectoriesAssign(traj, leftSingleSupportTime, "leftss", params);
    // Left apex
    trajParams.trajectoriesAssign(traj, leftApexTime, "leftapex", params);
    // Right double support
    trajParams.trajectoriesAssign(traj, rightDoubleSupportTime, "leftds", params, true, true);
    trajParams.trajectoriesAssign(traj, rightDoubleSupportTime, "leftds", params, false, true);
    // Right single support
    trajParams.trajectoriesAssign(traj, rightSingleSupportTime, "leftss", params, false, true);
    // Right apex
    trajParams.trajectoriesAssign(traj, rightApexTime, "leftapex", params, false, true);
    // Left double support 2
    trajParams.trajectoriesAssign(traj, leftDoubleSupport2Time, "leftds", params, true, false);

    return traj;
  };
}

TrajectoryGeneration::CheckParamsFunc TrajWalk::funcCheckParams(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> double {
    // Retrieve timing parameters
    return 0.0;
  };
}

TrajectoryGeneration::CheckStateFunc TrajWalk::funcCheckState(const TrajectoryParameters& trajParams)
{
  return DefaultCheckState;
}

TrajectoryGeneration::CheckDOFFunc TrajWalk::funcCheckDOF(const TrajectoryParameters& trajParams)
{
  return DefaultCheckDOF;
}

TrajectoryGeneration::ScoreFunc TrajWalk::funcScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncScore(trajParams);
}

TrajectoryGeneration::EndScoreFunc TrajWalk::funcEndScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncEndScore(trajParams);
}

TrajectoryGeneration::SaveFunc TrajWalk::funcSave(const TrajectoryParameters& trajParams)
{
  return DefaultFuncSave(trajParams);
}

}  // namespace Leph
