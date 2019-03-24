#include <iostream>
#include <Eigen/Dense>
#include "Spline/SmoothSpline.hpp"
#include "Utils/time.h"
#include "Spline/SplineContainer.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryDisplay.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"
#include "Utils/FileEigen.h"

int main()
{
  // Initialize the generator
  Leph::TrajectoryGeneration generator(Leph::SigmabanModel);

  // Joint Model
  Leph::JointModel jointModel;

  // Initial trajectory parameters
  generator.setInitialParameters([]() -> Eigen::VectorXd {
    Eigen::VectorXd params(100);
    params.setZero();

    params(0) = 0.2;
    params(1) = 0.5;
    params(2) = 0.6;
    params(3) = 0.8;

    params(4) = 0.0;
    params(5) = -0.04;
    params(6) = 0.28;

    params(22) = 0.0;
    params(23) = -0.04;
    params(24) = 0.28;
    params(28) = -0.01;
    params(29) = -0.14;
    params(30) = 0.05;

    params(27) = -0.4;
    params(54) = 0.4;

    params(49) = 0.0;
    params(50) = -0.04;
    params(51) = 0.28;

    params(69) = -6.0;

    params(72) = 0.0;
    params(73) = -0.04;
    params(74) = 0.28;
    params(78) = 0.0;
    params(79) = -0.14;

    params(92) = 0.0;
    params(93) = -0.0715;

    params(94) = -0.4;

    // Velocity
    params(99) = 1.0;

    return params;
  }());
  // generator.setInitialParameters(Leph::ReadEigenVector("../Data/trajKick_2016-07-12-11-00-29.params"));

  // Set Trajectory generation function
  generator.setTrajectoryGenerationFunc([](const Eigen::VectorXd& params) -> Leph::Trajectories {
    double trajectoryLength = 4.0;
    double ratioSwap1 = params(0);
    double ratioRetract = params(1);
    double ratioKick = params(2);
    double ratioSwap2 = params(3);

    double timeStart = 0.0;
    double timeSwap1 = ratioSwap1 * trajectoryLength;
    double timeRetract = ratioRetract * trajectoryLength;
    double timeKick = ratioKick * trajectoryLength;
    double timeSwap2 = ratioSwap2 * trajectoryLength;
    double timeEnd = trajectoryLength;

    // Start
    Eigen::Vector3d trunkPosAtStart(0.0, -0.0715, 0.29);
    Eigen::Vector3d trunkAngleAtStart(0.0, 0.0, 0.0);
    Eigen::Vector3d footPosAtStart(0.0, -0.1429, 0.0);

    // Swap1
    // Pos
    Eigen::Vector3d trunkPosAtSwap1(params(4), params(5), params(6));
    Eigen::Vector3d trunkAngleAtSwap1(params(7), params(8), params(9));
    Eigen::Vector3d footPosAtSwap1 = footPosAtStart;
    // Vel
    Eigen::Vector3d trunkPosVelAtSwap1(params(10), params(11), params(12));
    Eigen::Vector3d trunkAngleVelAtSwap1(params(13), params(14), params(15));
    Eigen::Vector3d footPosVelAtSwap1(0.0, 0.0, 0.0);
    // Acc
    Eigen::Vector3d trunkPosAccAtSwap1(params(16), params(17), params(18));
    Eigen::Vector3d trunkAngleAccAtSwap1(params(19), params(20), params(21));
    Eigen::Vector3d footPosAccAtSwap1(0.0, 0.0, 0.0);

    // Retract
    // Pos
    Eigen::Vector3d trunkPosAtRetract(params(22), params(23), params(24));
    Eigen::Vector3d trunkAngleAtRetract(params(25), params(26), params(27));
    Eigen::Vector3d footPosAtRetract(params(28), params(29), params(30));
    // Vel
    Eigen::Vector3d trunkPosVelAtRetract(params(31), params(32), params(33));
    Eigen::Vector3d trunkAngleVelAtRetract(params(34), params(35), params(36));
    Eigen::Vector3d footPosVelAtRetract(params(37), params(38), params(39));
    // Acc
    Eigen::Vector3d trunkPosAccAtRetract(params(40), params(41), params(42));
    Eigen::Vector3d trunkAngleAccAtRetract(params(43), params(44), params(45));
    Eigen::Vector3d footPosAccAtRetract(params(46), params(47), params(48));

    // Kick
    // Pos
    Eigen::Vector3d trunkPosAtKick(params(49), params(50), params(51));
    Eigen::Vector3d trunkAngleAtKick(params(52), params(53), params(54));
    Eigen::Vector3d footPosAtKick(0.05, -0.13, 0.075);
    // Vel
    Eigen::Vector3d trunkPosVelAtKick(params(55), params(56), params(57));
    Eigen::Vector3d trunkAngleVelAtKick(params(58), params(59), params(60));
    Eigen::Vector3d footPosVelAtKick(params(99), params(61), params(62));
    // Acc
    Eigen::Vector3d trunkPosAccAtKick(params(63), params(64), params(65));
    Eigen::Vector3d trunkAngleAccAtKick(params(66), params(67), params(68));
    Eigen::Vector3d footPosAccAtKick(params(69), params(70), params(71));

    // Swap2
    // Pos
    Eigen::Vector3d trunkPosAtSwap2(params(72), params(73), params(74));
    Eigen::Vector3d trunkAngleAtSwap2(params(75), params(76), params(77));
    Eigen::Vector3d footPosAtSwap2(params(78), params(79), 0.0);
    // Vel
    Eigen::Vector3d trunkPosVelAtSwap2(params(80), params(81), params(82));
    Eigen::Vector3d trunkAngleVelAtSwap2(params(83), params(84), params(85));
    Eigen::Vector3d footPosVelAtSwap2(0.0, 0.0, 0.0);
    // Acc
    Eigen::Vector3d trunkPosAccAtSwap2(params(86), params(87), params(88));
    Eigen::Vector3d trunkAngleAccAtSwap2(params(89), params(90), params(91));
    Eigen::Vector3d footPosAccAtSwap2(0.0, 0.0, 0.0);

    // End
    Eigen::Vector3d trunkPosAtEnd(params(92), params(93), 0.29);
    Eigen::Vector3d trunkAngleAtEnd(0.0, 0.0, 0.0);
    Eigen::Vector3d footPosAtEnd = footPosAtSwap2;

    // Foot orientation
    // Retract
    double footYawAtRetract = params(94);
    double footYawVelAtRetract = params(95);
    double footYawAccAtRetract = params(96);
    // Kick
    double footYawAtKick = 0.0;
    double footYawVelAtKick = params(97);
    double footYawAccAtKick = params(98);

    // Initialize state splines
    Leph::Trajectories traj = Leph::TrajectoriesInit();

    // Foot orientation
    traj.get("foot_axis_x").addPoint(timeStart, 0.0, 0.0);
    traj.get("foot_axis_x").addPoint(timeEnd, 0.0, 0.0);
    traj.get("foot_axis_y").addPoint(timeStart, 0.0, 0.0);
    traj.get("foot_axis_y").addPoint(timeEnd, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(timeStart, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(timeSwap1, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(timeRetract, footYawAtRetract, footYawVelAtRetract, footYawAccAtRetract);
    traj.get("foot_axis_z").addPoint(timeKick, footYawAtKick, footYawVelAtKick, footYawAccAtKick);
    traj.get("foot_axis_z").addPoint(timeSwap2, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(timeEnd, 0.0, 0.0);

    // Foot support
    traj.get("is_left_support_foot").addPoint(timeStart, 1.0, 0.0);
    traj.get("is_left_support_foot").addPoint(timeEnd, 1.0, 0.0);

    // Support phase
    traj.get("is_double_support").addPoint(timeStart, 1.0, 0.0);
    traj.get("is_double_support").addPoint(timeSwap1, 1.0, 0.0);
    traj.get("is_double_support").addPoint(timeSwap1, 0.0, 0.0);
    traj.get("is_double_support").addPoint(timeSwap2, 0.0, 0.0);
    traj.get("is_double_support").addPoint(timeSwap2, 1.0, 0.0);
    traj.get("is_double_support").addPoint(timeEnd, 1.0, 0.0);

    // Start
    traj.get("trunk_pos_x").addPoint(timeStart, trunkPosAtStart.x(), 0.0);
    traj.get("trunk_pos_y").addPoint(timeStart, trunkPosAtStart.y(), 0.0);
    traj.get("trunk_pos_z").addPoint(timeStart, trunkPosAtStart.z(), 0.0);
    traj.get("trunk_axis_x").addPoint(timeStart, trunkAngleAtStart.x(), 0.0);
    traj.get("trunk_axis_y").addPoint(timeStart, trunkAngleAtStart.y(), 0.0);
    traj.get("trunk_axis_z").addPoint(timeStart, trunkAngleAtStart.z(), 0.0);
    traj.get("foot_pos_x").addPoint(timeStart, footPosAtStart.x(), 0.0);
    traj.get("foot_pos_y").addPoint(timeStart, footPosAtStart.y(), 0.0);
    traj.get("foot_pos_z").addPoint(timeStart, footPosAtStart.z(), 0.0);
    // Swap1
    traj.get("trunk_pos_x").addPoint(timeSwap1, trunkPosAtSwap1.x(), trunkPosVelAtSwap1.x(), trunkPosAccAtSwap1.x());
    traj.get("trunk_pos_y").addPoint(timeSwap1, trunkPosAtSwap1.y(), trunkPosVelAtSwap1.y(), trunkPosAccAtSwap1.y());
    traj.get("trunk_pos_z").addPoint(timeSwap1, trunkPosAtSwap1.z(), trunkPosVelAtSwap1.z(), trunkPosAccAtSwap1.z());
    traj.get("trunk_axis_x")
        .addPoint(timeSwap1, trunkAngleAtSwap1.x(), trunkAngleVelAtSwap1.x(), trunkAngleAccAtSwap1.x());
    traj.get("trunk_axis_y")
        .addPoint(timeSwap1, trunkAngleAtSwap1.y(), trunkAngleVelAtSwap1.y(), trunkAngleAccAtSwap1.y());
    traj.get("trunk_axis_z")
        .addPoint(timeSwap1, trunkAngleAtSwap1.z(), trunkAngleVelAtSwap1.z(), trunkAngleAccAtSwap1.z());
    traj.get("foot_pos_x").addPoint(timeSwap1, footPosAtSwap1.x(), footPosVelAtSwap1.x(), footPosAccAtSwap1.x());
    traj.get("foot_pos_y").addPoint(timeSwap1, footPosAtSwap1.y(), footPosVelAtSwap1.y(), footPosAccAtSwap1.y());
    traj.get("foot_pos_z").addPoint(timeSwap1, footPosAtSwap1.z(), footPosVelAtSwap1.z(), footPosAccAtSwap1.z());
    // Retract
    traj.get("trunk_pos_x")
        .addPoint(timeRetract, trunkPosAtRetract.x(), trunkPosVelAtRetract.x(), trunkPosAccAtRetract.x());
    traj.get("trunk_pos_y")
        .addPoint(timeRetract, trunkPosAtRetract.y(), trunkPosVelAtRetract.y(), trunkPosAccAtRetract.y());
    traj.get("trunk_pos_z")
        .addPoint(timeRetract, trunkPosAtRetract.z(), trunkPosVelAtRetract.z(), trunkPosAccAtRetract.z());
    traj.get("trunk_axis_x")
        .addPoint(timeRetract, trunkAngleAtRetract.x(), trunkAngleVelAtRetract.x(), trunkAngleAccAtRetract.x());
    traj.get("trunk_axis_y")
        .addPoint(timeRetract, trunkAngleAtRetract.y(), trunkAngleVelAtRetract.y(), trunkAngleAccAtRetract.y());
    traj.get("trunk_axis_z")
        .addPoint(timeRetract, trunkAngleAtRetract.z(), trunkAngleVelAtRetract.z(), trunkAngleAccAtRetract.z());
    traj.get("foot_pos_x")
        .addPoint(timeRetract, footPosAtRetract.x(), footPosVelAtRetract.x(), footPosAccAtRetract.x());
    traj.get("foot_pos_y")
        .addPoint(timeRetract, footPosAtRetract.y(), footPosVelAtRetract.y(), footPosAccAtRetract.y());
    traj.get("foot_pos_z")
        .addPoint(timeRetract, footPosAtRetract.z(), footPosVelAtRetract.z(), footPosAccAtRetract.z());
    // Kick
    traj.get("trunk_pos_x").addPoint(timeKick, trunkPosAtKick.x(), trunkPosVelAtKick.x(), trunkPosAccAtKick.x());
    traj.get("trunk_pos_y").addPoint(timeKick, trunkPosAtKick.y(), trunkPosVelAtKick.y(), trunkPosAccAtKick.y());
    traj.get("trunk_pos_z").addPoint(timeKick, trunkPosAtKick.z(), trunkPosVelAtKick.z(), trunkPosAccAtKick.z());
    traj.get("trunk_axis_x").addPoint(timeKick, trunkAngleAtKick.x(), trunkAngleVelAtKick.x(), trunkAngleAccAtKick.x());
    traj.get("trunk_axis_y").addPoint(timeKick, trunkAngleAtKick.y(), trunkAngleVelAtKick.y(), trunkAngleAccAtKick.y());
    traj.get("trunk_axis_z").addPoint(timeKick, trunkAngleAtKick.z(), trunkAngleVelAtKick.z(), trunkAngleAccAtKick.z());
    traj.get("foot_pos_x").addPoint(timeKick, footPosAtKick.x(), footPosVelAtKick.x(), footPosAccAtKick.x());
    traj.get("foot_pos_y").addPoint(timeKick, footPosAtKick.y(), footPosVelAtKick.y(), footPosAccAtKick.y());
    traj.get("foot_pos_z").addPoint(timeKick, footPosAtKick.z(), footPosVelAtKick.z(), footPosAccAtKick.z());
    // Swap2
    traj.get("trunk_pos_x").addPoint(timeSwap2, trunkPosAtSwap2.x(), trunkPosVelAtSwap2.x(), trunkPosAccAtSwap2.x());
    traj.get("trunk_pos_y").addPoint(timeSwap2, trunkPosAtSwap2.y(), trunkPosVelAtSwap2.y(), trunkPosAccAtSwap2.y());
    traj.get("trunk_pos_z").addPoint(timeSwap2, trunkPosAtSwap2.z(), trunkPosVelAtSwap2.z(), trunkPosAccAtSwap2.z());
    traj.get("trunk_axis_x")
        .addPoint(timeSwap2, trunkAngleAtSwap2.x(), trunkAngleVelAtSwap2.x(), trunkAngleAccAtSwap2.x());
    traj.get("trunk_axis_y")
        .addPoint(timeSwap2, trunkAngleAtSwap2.y(), trunkAngleVelAtSwap2.y(), trunkAngleAccAtSwap2.y());
    traj.get("trunk_axis_z")
        .addPoint(timeSwap2, trunkAngleAtSwap2.z(), trunkAngleVelAtSwap2.z(), trunkAngleAccAtSwap2.z());
    traj.get("foot_pos_x").addPoint(timeSwap2, footPosAtSwap2.x(), footPosVelAtSwap2.x(), footPosAccAtSwap2.x());
    traj.get("foot_pos_y").addPoint(timeSwap2, footPosAtSwap2.y(), footPosVelAtSwap2.y(), footPosAccAtSwap2.y());
    traj.get("foot_pos_z").addPoint(timeSwap2, footPosAtSwap2.z(), footPosVelAtSwap2.z(), footPosAccAtSwap2.z());
    // End
    traj.get("trunk_pos_x").addPoint(timeEnd, trunkPosAtEnd.x(), 0.0);
    traj.get("trunk_pos_y").addPoint(timeEnd, trunkPosAtEnd.y(), 0.0);
    traj.get("trunk_pos_z").addPoint(timeEnd, trunkPosAtEnd.z(), 0.0);
    traj.get("trunk_axis_x").addPoint(timeEnd, trunkAngleAtEnd.x(), 0.0);
    traj.get("trunk_axis_y").addPoint(timeEnd, trunkAngleAtEnd.y(), 0.0);
    traj.get("trunk_axis_z").addPoint(timeEnd, trunkAngleAtEnd.z(), 0.0);
    traj.get("foot_pos_x").addPoint(timeEnd, footPosAtEnd.x(), 0.0);
    traj.get("foot_pos_y").addPoint(timeEnd, footPosAtEnd.y(), 0.0);
    traj.get("foot_pos_z").addPoint(timeEnd, footPosAtEnd.z(), 0.0);

    return traj;
  });

  // Set parameters bound function
  generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
    // Check support ratio bound
    if (params(0) <= 0.1)
    {
      return 1000.0 - 1000.0 * (params(0) - 0.1);
    }
    if (params(0) >= 0.9)
    {
      return 1000.0 + 1000.0 * (params(0) - 0.9);
    }
    if (params(1) - params(0) < 0.05)
    {
      return 1000.0 + 1000.0 * (0.05 - params(1) + params(0));
    }
    if (params(2) - params(1) < 0.05)
    {
      return 1000.0 + 1000.0 * (0.05 - params(2) + params(1));
    }
    if (params(3) > 0.9)
    {
      return 1000.0 + 1000.0 * (params(3) - 0.9);
    }
    return 0.0;
  });
  // Set default Cartesian state and Joint DOF bounds
  generator.setCheckStateFunc(Leph::DefaultCheckState);
  generator.setCheckDOFFunc(Leph::DefaultCheckDOF);
  // Set trajectory scoring function
  generator.setScoreFunc([](double t, Leph::HumanoidFixedModel& model,
                            const std::map<std::string, Leph::JointModel>& joints, const Eigen::VectorXd& torques,
                            const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq, bool isDoubleSupport,
                            Leph::HumanoidFixedModel::SupportFoot supportFoot, std::vector<double>& data) -> double {
    (void)t;
    (void)model;
    (void)ddq;
    (void)supportFoot;
    double cost = 0.0;

    // Torques
    Eigen::VectorXd tmpTorques = torques;
    tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;
    cost += 0.05 * tmpTorques.norm();

    if (data.size() == 0)
    {
      data.push_back(0.0);
      data.push_back(0.0);
    }

    // Maximum voltage
    for (const std::string& name : Leph::NamesDOF)
    {
      size_t index = model.get().getDOFIndex(name);
      double volt = joints.at(name).computeElectricTension(dq(index), ddq(index), torques(index));
      // Maximum voltage
      if (data[0] < volt)
      {
        data[0] = volt;
      }
    }

    // Maximum ZMP
    Eigen::Vector3d zmp = model.zeroMomentPoint("origin", dq, ddq, false);
    zmp.z() = 0.0;
    if (data[1] < zmp.lpNorm<Eigen::Infinity>())
    {
      data[1] = zmp.lpNorm<Eigen::Infinity>();
    }

    return cost;
  });
  generator.setEndScoreFunc([&jointModel](const Eigen::VectorXd& params, const Leph::Trajectories& traj, double score,
                                          std::vector<double>& data, bool verbose) -> double {
    (void)params;
    (void)traj;
    (void)score;
    (void)verbose;
    double cost = 0.0;
    if (data[0] > jointModel.getMaxVoltage())
    {
      cost += 5.0 + 5.0 * data[0];
    }
    if (data[1] > 0.02)
    {
      cost += 5.0 + 500.0 * data[1];
    }
    else
    {
      cost += 5.0 + 50.0 * data[1];
    }
    cost += 100.0 / params(99);
    return cost;
  });

  // Display initial trajectory
  generator.scoreTrajectory(generator.initialParameters(), true);
  TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

  // Target filename
  std::string filename = "/tmp/trajKick_" + Leph::currentDate();
  // Run the CMA-ES optimization
  generator.runOptimization(2000, 10, filename, 20, -1.0);

  // Display found trajectory
  TrajectoriesDisplay(generator.bestTrajectories());

  return 0;
}
