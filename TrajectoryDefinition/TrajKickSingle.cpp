#include <iostream>
#include "TrajectoryDefinition/TrajKickSingle.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

namespace Leph
{
void TrajKickSingle::initializeParameters(TrajectoryParameters& trajParams, bool isFwd)
{
  // Total time length
  trajParams.set("time_length", false) = 1.5;
  // Time ratio for control points
  trajParams.set("time_ratio_before1", true) = 0.3;
  trajParams.set("time_ratio_before2", true) = 0.4;
  trajParams.set("time_ratio_contact", true) = 0.5;
  trajParams.set("time_ratio_after", true) = 0.7;

  // Kick configuration
  trajParams.set("kick_x", isFwd) = 0.06;
  trajParams.set("kick_y", isFwd) = -0.12;
  trajParams.set("kick_z", isFwd) = 0.06;
  trajParams.set("kick_vel", isFwd) = 1.0;

  // Position at contact
  trajParams.set("contact_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("contact_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("contact_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("contact_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("contact_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("contact_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.cpy("contact_pos_foot_pos_x", "kick_x");
  trajParams.cpy("contact_pos_foot_pos_y", "kick_y");
  trajParams.cpy("contact_pos_foot_pos_z", "kick_z");
  trajParams.set("contact_pos_foot_axis_x", false) = 0.0;
  trajParams.set("contact_pos_foot_axis_y", isFwd) = 0.0;
  trajParams.set("contact_pos_foot_axis_z", false) = 0.0;
  // Velocity at contact
  trajParams.set("contact_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("contact_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("contact_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("contact_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("contact_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("contact_vel_trunk_axis_z", true) = 0.0;
  trajParams.cpy("contact_vel_foot_pos_x", "kick_vel");
  trajParams.set("contact_vel_foot_pos_y", isFwd) = 0.0;
  trajParams.set("contact_vel_foot_pos_z", isFwd) = 0.0;
  trajParams.set("contact_vel_foot_axis_x", false) = 0.0;
  trajParams.set("contact_vel_foot_axis_y", isFwd) = 0.0;
  trajParams.set("contact_vel_foot_axis_z", false) = 0.0;
  // Acceleration at contact
  trajParams.set("contact_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("contact_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("contact_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("contact_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("contact_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("contact_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("contact_acc_foot_pos_x", true) = 0.0;
  trajParams.set("contact_acc_foot_pos_y", isFwd) = 0.0;
  trajParams.set("contact_acc_foot_pos_z", true) = 0.0;
  trajParams.set("contact_acc_foot_axis_x", false) = 0.0;
  trajParams.set("contact_acc_foot_axis_y", isFwd) = 0.0;
  trajParams.set("contact_acc_foot_axis_z", false) = 0.0;

  // Position at before1
  trajParams.set("before1_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("before1_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("before1_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("before1_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("before1_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("before1_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("before1_pos_foot_pos_x", true) = trajParams.get("static_single_pos_foot_pos_x");
  trajParams.set("before1_pos_foot_pos_y", true) = trajParams.get("kick_y");
  trajParams.set("before1_pos_foot_pos_z", true) = trajParams.get("static_single_pos_foot_pos_z");
  trajParams.set("before1_pos_foot_axis_x", false) = 0.0;
  trajParams.set("before1_pos_foot_axis_y", isFwd) = 0.0;
  trajParams.set("before1_pos_foot_axis_z", false) = 0.0;
  // Velocity at before1
  trajParams.set("before1_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("before1_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("before1_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("before1_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("before1_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("before1_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("before1_vel_foot_pos_x", isFwd) = 0.0;
  trajParams.set("before1_vel_foot_pos_y", isFwd) = 0.0;
  trajParams.set("before1_vel_foot_pos_z", isFwd) = 0.0;
  trajParams.set("before1_vel_foot_axis_x", false) = 0.0;
  trajParams.set("before1_vel_foot_axis_y", isFwd) = 0.0;
  trajParams.set("before1_vel_foot_axis_z", false) = 0.0;
  // Acceleration at before1
  trajParams.set("before1_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("before1_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("before1_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("before1_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("before1_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("before1_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("before1_acc_foot_pos_x", true) = 0.0;
  trajParams.set("before1_acc_foot_pos_y", isFwd) = 0.0;
  trajParams.set("before1_acc_foot_pos_z", true) = 0.0;
  trajParams.set("before1_acc_foot_axis_x", false) = 0.0;
  trajParams.set("before1_acc_foot_axis_y", isFwd) = 0.0;
  trajParams.set("before1_acc_foot_axis_z", false) = 0.0;

  // Position at before2
  trajParams.set("before2_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("before2_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("before2_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("before2_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("before2_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("before2_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("before2_pos_foot_pos_x", true) = trajParams.get("static_single_pos_foot_pos_x");
  trajParams.cpy("before2_pos_foot_pos_y", "kick_y");
  trajParams.set("before2_pos_foot_pos_z", true) = trajParams.get("kick_z");
  trajParams.set("before2_pos_foot_axis_x", false) = 0.0;
  trajParams.set("before2_pos_foot_axis_y", isFwd) = 0.0;
  trajParams.set("before2_pos_foot_axis_z", false) = 0.0;
  // Velocity at before2
  trajParams.set("before2_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("before2_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("before2_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("before2_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("before2_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("before2_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("before2_vel_foot_pos_x", true) = 0.0;
  trajParams.set("before2_vel_foot_pos_y", false) = 0.0;
  trajParams.set("before2_vel_foot_pos_z", true) = 0.0;
  trajParams.set("before2_vel_foot_axis_x", false) = 0.0;
  trajParams.set("before2_vel_foot_axis_y", isFwd) = 0.0;
  trajParams.set("before2_vel_foot_axis_z", false) = 0.0;
  // Acceleration at before2
  trajParams.set("before2_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("before2_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("before2_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("before2_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("before2_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("before2_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("before2_acc_foot_pos_x", true) = 0.0;
  trajParams.set("before2_acc_foot_pos_y", false) = 0.0;
  trajParams.set("before2_acc_foot_pos_z", true) = 0.0;
  trajParams.set("before2_acc_foot_axis_x", false) = 0.0;
  trajParams.set("before2_acc_foot_axis_y", isFwd) = 0.0;
  trajParams.set("before2_acc_foot_axis_z", false) = 0.0;

  // Position at after
  trajParams.set("after_pos_trunk_pos_x", true) = trajParams.get("static_single_pos_trunk_pos_x");
  trajParams.set("after_pos_trunk_pos_y", true) = trajParams.get("static_single_pos_trunk_pos_y");
  trajParams.set("after_pos_trunk_pos_z", true) = trajParams.get("static_single_pos_trunk_pos_z");
  trajParams.set("after_pos_trunk_axis_x", true) = trajParams.get("static_single_pos_trunk_axis_x");
  trajParams.set("after_pos_trunk_axis_y", true) = trajParams.get("static_single_pos_trunk_axis_y");
  trajParams.set("after_pos_trunk_axis_z", true) = trajParams.get("static_single_pos_trunk_axis_z");
  trajParams.set("after_pos_foot_pos_x", true) = trajParams.get("static_single_pos_foot_pos_x");
  trajParams.cpy("after_pos_foot_pos_y", "kick_y");
  trajParams.set("after_pos_foot_pos_z", true) = trajParams.get("static_single_pos_foot_pos_z");
  trajParams.set("after_pos_foot_axis_x", false) = 0.0;
  trajParams.set("after_pos_foot_axis_y", isFwd) = 0.0;
  trajParams.set("after_pos_foot_axis_z", false) = 0.0;
  // Velocity at after
  trajParams.set("after_vel_trunk_pos_x", true) = 0.0;
  trajParams.set("after_vel_trunk_pos_y", true) = 0.0;
  trajParams.set("after_vel_trunk_pos_z", true) = 0.0;
  trajParams.set("after_vel_trunk_axis_x", true) = 0.0;
  trajParams.set("after_vel_trunk_axis_y", true) = 0.0;
  trajParams.set("after_vel_trunk_axis_z", true) = 0.0;
  trajParams.set("after_vel_foot_pos_x", true) = 0.0;
  trajParams.set("after_vel_foot_pos_y", false) = 0.0;
  trajParams.set("after_vel_foot_pos_z", true) = 0.0;
  trajParams.set("after_vel_foot_axis_x", false) = 0.0;
  trajParams.set("after_vel_foot_axis_y", isFwd) = 0.0;
  trajParams.set("after_vel_foot_axis_z", false) = 0.0;
  // Acceleration at after
  trajParams.set("after_acc_trunk_pos_x", true) = 0.0;
  trajParams.set("after_acc_trunk_pos_y", true) = 0.0;
  trajParams.set("after_acc_trunk_pos_z", true) = 0.0;
  trajParams.set("after_acc_trunk_axis_x", true) = 0.0;
  trajParams.set("after_acc_trunk_axis_y", true) = 0.0;
  trajParams.set("after_acc_trunk_axis_z", true) = 0.0;
  trajParams.set("after_acc_foot_pos_x", true) = 0.0;
  trajParams.set("after_acc_foot_pos_y", isFwd) = 0.0;
  trajParams.set("after_acc_foot_pos_z", true) = 0.0;
  trajParams.set("after_acc_foot_axis_x", false) = 0.0;
  trajParams.set("after_acc_foot_axis_y", isFwd) = 0.0;
  trajParams.set("after_acc_foot_axis_z", false) = 0.0;
}

TrajectoryGeneration::GenerationFunc TrajKickSingle::funcGeneration(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> Trajectories {
    // Retrieve timing parameters
    double endTime = trajParams.get("time_length", params);
    double before1Time = trajParams.get("time_ratio_before1", params) * endTime;
    double before2Time = trajParams.get("time_ratio_before2", params) * endTime;
    double contactTime = trajParams.get("time_ratio_contact", params) * endTime;
    double afterTime = trajParams.get("time_ratio_after", params) * endTime;

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
    // Pre Kick 1
    trajParams.trajectoriesAssign(traj, before1Time, "before1", params);
    // Pre Kick 2
    trajParams.trajectoriesAssign(traj, before2Time, "before2", params);
    // Kick contact
    trajParams.trajectoriesAssign(traj, contactTime, "contact", params);
    // Post Kick
    trajParams.trajectoriesAssign(traj, afterTime, "after", params);
    // Ending in single support pose
    trajParams.trajectoriesAssign(traj, endTime, "static_single", params);

    return traj;
  };
}

TrajectoryGeneration::CheckParamsFunc TrajKickSingle::funcCheckParams(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params) -> double {
    // Retrieve timing parameters
    double before1Ratio = trajParams.get("time_ratio_before1", params);
    double before2Ratio = trajParams.get("time_ratio_before2", params);
    double contactRatio = trajParams.get("time_ratio_contact", params);
    double afterRatio = trajParams.get("time_ratio_after", params);
    double timeLength = trajParams.get("time_length", params);
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
    if (before1Ratio <= 0.1)
    {
      return 1000.0 - 1000.0 * (before1Ratio - 0.1);
    }
    if (before1Ratio >= 0.9)
    {
      return 1000.0 + 1000.0 * (before1Ratio - 0.9);
    }
    if (before2Ratio <= 0.1)
    {
      return 1000.0 - 1000.0 * (before2Ratio - 0.1);
    }
    if (before2Ratio >= 0.9)
    {
      return 1000.0 + 1000.0 * (before2Ratio - 0.9);
    }
    if (contactRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (contactRatio - 0.1);
    }
    if (contactRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (contactRatio - 0.9);
    }
    if (afterRatio <= 0.1)
    {
      return 1000.0 - 1000.0 * (afterRatio - 0.1);
    }
    if (afterRatio >= 0.9)
    {
      return 1000.0 + 1000.0 * (afterRatio - 0.9);
    }
    if (before1Ratio + 0.05 > before2Ratio)
    {
      return 1000.0 - 1000.0 * (before2Ratio - before1Ratio - 0.05);
    }
    if (before2Ratio + 0.05 > contactRatio)
    {
      return 1000.0 - 1000.0 * (contactRatio - before2Ratio - 0.05);
    }
    if (contactRatio + 0.05 > afterRatio)
    {
      return 1000.0 - 1000.0 * (afterRatio - contactRatio - 0.05);
    }
    return 0.0;
  };
}

TrajectoryGeneration::CheckStateFunc TrajKickSingle::funcCheckState(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                       const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                       const Eigen::Vector3d& footAxis) -> double {
    double cost = 0.0;
    // Retrieve kick velocity
    double contactVel = trajParams.get("kick_vel", params);
    // Check that the foot is not colliding
    // the ball before contact time
    double contactTime = trajParams.get("time_length", params) * trajParams.get("time_ratio_contact", params);
    double contactPos = trajParams.get("contact_pos_foot_pos_x", params);
    // Penalize forward kick
    if (t < contactTime - 0.01 && contactVel >= 0.0 && footPos.x() > contactPos)
    {
      cost += 1000.0 + 1000.0 * (footPos.x() - contactPos);
    }
    // Penalize backward kick
    if (t < contactTime - 0.01 && contactVel < 0.0 && footPos.x() < contactPos)
    {
      cost += 1000.0 - 1000.0 * (footPos.x() - contactPos);
    }
    // Forward to default state check
    cost += DefaultCheckState(params, t, trunkPos, trunkAxis, footPos, footAxis);
    return cost;
  };
}

TrajectoryGeneration::CheckDOFFunc TrajKickSingle::funcCheckDOF(const TrajectoryParameters& trajParams)
{
  return DefaultCheckDOF;
}

TrajectoryGeneration::ScoreFunc TrajKickSingle::funcScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncScore(trajParams);
}

TrajectoryGeneration::EndScoreFunc TrajKickSingle::funcEndScore(const TrajectoryParameters& trajParams)
{
  return DefaultFuncEndScore(trajParams);
}

TrajectoryGeneration::ScoreSimFunc TrajKickSingle::funcScoreSim(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params, double t, HumanoidSimulation& sim,
                       std::vector<double>& data) -> double {
    if (data.size() == 0)
    {
      // Max X vel
      data.push_back(0.0);
      // X pos at max vel
      data.push_back(0.0);
      // Y pos at max vel
      data.push_back(0.0);
      // Z pos at max vel
      data.push_back(0.0);
    }

    // Retrieve kick vel sign
    double kickSign = (trajParams.get("kick_vel", params) >= 0.0 ? 1.0 : -1.0);
    // Compute foot velocity
    Eigen::VectorXd footVel = sim.model().pointVelocity("right_foot_tip", "left_foot_tip", sim.velocities());
    // Save maximum velocity
    if (data[0] < kickSign * footVel(3))
    {
      Eigen::Vector3d footPos = sim.model().position("right_foot_tip", "left_foot_tip");
      data[0] = kickSign * footVel(3);
      data[1] = footPos.x();
      data[2] = footPos.y();
      data[3] = footPos.z();
    }

    // Compute support left foot
    // euler angle orientation
    Eigen::Matrix3d footMat = sim.model().orientation("left_foot_tip", "origin");
    Eigen::Vector3d angles = Eigen::Vector3d::Zero();
    // Retrieve YawPitchRoll euler angles from rotation matrix
    //(Manual computing without singular check seems better than
    // Eigen euler angles and with better range)
    // Roll
    angles(0) = atan2(footMat(1, 2), footMat(2, 2));
    // Pitch
    angles(1) = atan2(-footMat(0, 2), sqrt(footMat(0, 0) * footMat(0, 0) + footMat(0, 1) * footMat(0, 1)));

    // Penalize support foot rotation
    return 0.05 * (fabs(angles(0)) + fabs(angles(1)));
  };
}

TrajectoryGeneration::EndScoreSimFunc TrajKickSingle::funcEndScoreSim(const TrajectoryParameters& trajParams)
{
  return [&trajParams](const Eigen::VectorXd& params, const Trajectories& traj, double score, std::vector<double>& data,
                       bool verbose) -> double {
    // Empty case
    if (data.size() != 4)
    {
      return 0.0;
    }
    if (verbose)
    {
      std::cout << "Score=" << score << " MaxVel=" << data[0] << " PosX=" << data[1] << " PosY=" << data[2]
                << " PosZ=" << data[3] << " EndScore=" << 1.0 / data[0] << std::endl;
    }
    return 1.0 / data[0];
  };
}

TrajectoryGeneration::SaveFunc TrajKickSingle::funcSave(const TrajectoryParameters& trajParams)
{
  return DefaultFuncSave(trajParams);
}

}  // namespace Leph
