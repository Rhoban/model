#include <iostream>
#include <Eigen/Dense>
#include "Spline/SmoothSpline.hpp"
#include "Spline/FittedSpline.hpp"
#include "Utils/time.h"
#include "Spline/SplineContainer.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryDisplay.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

/**
 * Ending precompute optimal state
 */
static Eigen::Vector3d targetTrunkPos()
{
  return Eigen::Vector3d(-0.00557785331559037, -0.0115849568418458, 0.28);
}
static Eigen::Vector3d targetTrunkAxis()
{
  return Eigen::Vector3d(-0.672036398746933, 0.0743358280850477, 0.0028323027017884);
}
static Eigen::Vector3d targetFootPos()
{
  return Eigen::Vector3d(0.0208647084129351, -0.095, 0.0591693358237435);
}

/**
 * Return initial state
 */
static Eigen::Vector3d initTrunkPos()
{
  return Eigen::Vector3d(0.0, -0.05, 0.20);
}
static Eigen::Vector3d initTrunkAxis()
{
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}
static Eigen::Vector3d initFootPos()
{
  return Eigen::Vector3d(0.0, -0.1, 0.0);
}

void generateLegLift()
{
  // Initialize the generator
  Leph::TrajectoryGeneration generator(Leph::SigmabanModel);

  // Initial trajectory parameters
  generator.setInitialParameters([]() -> Eigen::VectorXd {
    Eigen::VectorXd params(43);
    // Double support ratio
    params(0) = 0.5;
    // Support swap trunk pos
    params(1) = 0.5 * initTrunkPos().x() + 0.5 * targetTrunkPos().x();
    params(2) = 0.5 * initTrunkPos().y() + 0.5 * targetTrunkPos().y();
    params(3) = 0.5 * initTrunkPos().z() + 0.5 * targetTrunkPos().z();
    // Support swap trunk vel
    params(4) = 0.0;
    params(5) = 0.0;
    params(6) = 0.0;
    // Support swap trunk axis
    params(7) = 0.5 * initTrunkAxis().x() + 0.5 * targetTrunkAxis().x();
    params(8) = 0.5 * initTrunkAxis().y() + 0.5 * targetTrunkAxis().y();
    params(9) = 0.5 * initTrunkAxis().z() + 0.5 * targetTrunkAxis().z();
    // Support swap trunk vel
    params(10) = 0.0;
    params(11) = 0.0;
    params(12) = 0.0;

    // Middle 1 trunk pos
    params(13) = 0.75 * initTrunkPos().x() + 0.25 * targetTrunkPos().x();
    params(14) = 0.75 * initTrunkPos().y() + 0.25 * targetTrunkPos().y();
    params(15) = 0.75 * initTrunkPos().z() + 0.25 * targetTrunkPos().z();
    // Middle 1 trunk vel
    params(16) = 0.0;
    params(17) = 0.0;
    params(18) = 0.0;
    // Middle 1 trunk axis
    params(19) = 0.75 * initTrunkAxis().x() + 0.25 * targetTrunkAxis().x();
    params(20) = 0.75 * initTrunkAxis().y() + 0.25 * targetTrunkAxis().y();
    params(21) = 0.75 * initTrunkAxis().z() + 0.25 * targetTrunkAxis().z();
    // Middle 1 trunk vel
    params(22) = 0.0;
    params(23) = 0.0;
    params(24) = 0.0;

    // Middle 2 trunk pos
    params(25) = 0.25 * initTrunkPos().x() + 0.75 * targetTrunkPos().x();
    params(26) = 0.25 * initTrunkPos().y() + 0.75 * targetTrunkPos().y();
    params(27) = 0.25 * initTrunkPos().z() + 0.75 * targetTrunkPos().z();
    // Middle 2 trunk vel
    params(28) = 0.0;
    params(29) = 0.0;
    params(30) = 0.0;
    // Middle 2 trunk axis
    params(31) = 0.25 * initTrunkAxis().x() + 0.75 * targetTrunkAxis().x();
    params(32) = 0.25 * initTrunkAxis().y() + 0.75 * targetTrunkAxis().y();
    params(33) = 0.25 * initTrunkAxis().z() + 0.75 * targetTrunkAxis().z();
    // Middle 2 trunk vel
    params(34) = 0.0;
    params(35) = 0.0;
    params(36) = 0.0;
    // Middle 2 foot pos
    params(37) = 0.5 * initFootPos().x() + 0.5 * targetFootPos().x();
    params(38) = 0.5 * initFootPos().y() + 0.5 * targetFootPos().y();
    params(39) = 0.5 * initFootPos().z() + 0.5 * targetFootPos().z();
    // Middle 2 foot vel
    params(40) = 0.0;
    params(41) = 0.0;
    params(42) = 0.0;

    return params;
  }());

  // Set Trajectory generation function
  generator.setTrajectoryGenerationFunc([](const Eigen::VectorXd& params) -> Leph::Trajectories {
    // Extract parameters
    double endTrajectoryTime = 3.0;
    double ratioSwapSupport = params(0);
    double swapSupportTime = ratioSwapSupport * endTrajectoryTime;
    Eigen::Vector3d trunkPosAtSwap = params.segment<3>(1);
    Eigen::Vector3d trunkPosVelAtSwap = params.segment<3>(4);
    Eigen::Vector3d trunkAngleAtSwap = params.segment<3>(7);
    Eigen::Vector3d trunkAngleVelAtSwap = params.segment<3>(10);
    Eigen::Vector3d footPosAtSwap = initFootPos();

    double middle1Time = 0.5 * ratioSwapSupport * endTrajectoryTime;
    Eigen::Vector3d trunkPosAtMiddle1 = params.segment<3>(13);
    Eigen::Vector3d trunkPosVelAtMiddle1 = params.segment<3>(16);
    Eigen::Vector3d trunkAngleAtMiddle1 = params.segment<3>(19);
    Eigen::Vector3d trunkAngleVelAtMiddle1 = params.segment<3>(22);

    double middle2Time = (0.5 * ratioSwapSupport + 0.5) * endTrajectoryTime;
    Eigen::Vector3d trunkPosAtMiddle2 = params.segment<3>(25);
    Eigen::Vector3d trunkPosVelAtMiddle2 = params.segment<3>(28);
    Eigen::Vector3d trunkAngleAtMiddle2 = params.segment<3>(31);
    Eigen::Vector3d trunkAngleVelAtMiddle2 = params.segment<3>(34);
    Eigen::Vector3d footPosAtMiddle2 = params.segment<3>(37);
    Eigen::Vector3d footPosVelAtMiddle2 = params.segment<3>(40);

    // Initialize state splines
    Leph::Trajectories traj = Leph::TrajectoriesInit();

    // Foot orientation
    traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
    traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
    traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
    traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

    // Foot support
    traj.get("is_left_support_foot").addPoint(0.0, 1.0, 0.0);
    traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0, 0.0);

    // Support phase
    traj.get("is_double_support").addPoint(0.0, 1.0, 0.0);
    traj.get("is_double_support").addPoint(swapSupportTime, 1.0, 0.0);
    traj.get("is_double_support").addPoint(swapSupportTime, 0.0, 0.0);
    traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);

    // Starting double support pose
    traj.get("trunk_pos_x").addPoint(0.0, initTrunkPos().x(), 0.0);
    traj.get("trunk_pos_y").addPoint(0.0, initTrunkPos().y(), 0.0);
    traj.get("trunk_pos_z").addPoint(0.0, initTrunkPos().z(), 0.0);
    traj.get("trunk_axis_x").addPoint(0.0, initTrunkAxis().x(), 0.0);
    traj.get("trunk_axis_y").addPoint(0.0, initTrunkAxis().y(), 0.0);
    traj.get("trunk_axis_z").addPoint(0.0, initTrunkAxis().z(), 0.0);
    traj.get("foot_pos_x").addPoint(0.0, initFootPos().x(), 0.0);
    traj.get("foot_pos_y").addPoint(0.0, initFootPos().y(), 0.0);
    traj.get("foot_pos_z").addPoint(0.0, initFootPos().z(), 0.0);

    // Double support phase middle
    traj.get("trunk_pos_x").addPoint(middle1Time, trunkPosAtMiddle1.x(), trunkPosVelAtMiddle1.x());
    traj.get("trunk_pos_y").addPoint(middle1Time, trunkPosAtMiddle1.y(), trunkPosVelAtMiddle1.y());
    traj.get("trunk_pos_z").addPoint(middle1Time, trunkPosAtMiddle1.z(), trunkPosVelAtMiddle1.z());
    traj.get("trunk_axis_x").addPoint(middle1Time, trunkAngleAtMiddle1.x(), trunkAngleVelAtMiddle1.x());
    traj.get("trunk_axis_y").addPoint(middle1Time, trunkAngleAtMiddle1.y(), trunkAngleVelAtMiddle1.y());
    traj.get("trunk_axis_z").addPoint(middle1Time, trunkAngleAtMiddle1.z(), trunkAngleVelAtMiddle1.z());

    // Double to single support phase
    traj.get("trunk_pos_x").addPoint(swapSupportTime, trunkPosAtSwap.x(), trunkPosVelAtSwap.x());
    traj.get("trunk_pos_y").addPoint(swapSupportTime, trunkPosAtSwap.y(), trunkPosVelAtSwap.y());
    traj.get("trunk_pos_z").addPoint(swapSupportTime, trunkPosAtSwap.z(), trunkPosVelAtSwap.z());
    traj.get("trunk_axis_x").addPoint(swapSupportTime, trunkAngleAtSwap.x(), trunkAngleVelAtSwap.x());
    traj.get("trunk_axis_y").addPoint(swapSupportTime, trunkAngleAtSwap.y(), trunkAngleVelAtSwap.y());
    traj.get("trunk_axis_z").addPoint(swapSupportTime, trunkAngleAtSwap.z(), trunkAngleVelAtSwap.z());
    traj.get("foot_pos_x").addPoint(swapSupportTime, footPosAtSwap.x(), 0.0);
    traj.get("foot_pos_y").addPoint(swapSupportTime, footPosAtSwap.y(), 0.0);
    traj.get("foot_pos_z").addPoint(swapSupportTime, footPosAtSwap.z(), 0.0);

    // Double support phase middle
    traj.get("trunk_pos_x").addPoint(middle2Time, trunkPosAtMiddle2.x(), trunkPosVelAtMiddle2.x());
    traj.get("trunk_pos_y").addPoint(middle2Time, trunkPosAtMiddle2.y(), trunkPosVelAtMiddle2.y());
    traj.get("trunk_pos_z").addPoint(middle2Time, trunkPosAtMiddle2.z(), trunkPosVelAtMiddle2.z());
    traj.get("trunk_axis_x").addPoint(middle2Time, trunkAngleAtMiddle2.x(), trunkAngleVelAtMiddle2.x());
    traj.get("trunk_axis_y").addPoint(middle2Time, trunkAngleAtMiddle2.y(), trunkAngleVelAtMiddle2.y());
    traj.get("trunk_axis_z").addPoint(middle2Time, trunkAngleAtMiddle2.z(), trunkAngleVelAtMiddle2.z());
    traj.get("foot_pos_x").addPoint(middle2Time, footPosAtMiddle2.x(), footPosVelAtMiddle2.x());
    traj.get("foot_pos_y").addPoint(middle2Time, footPosAtMiddle2.y(), footPosVelAtMiddle2.y());
    traj.get("foot_pos_z").addPoint(middle2Time, footPosAtMiddle2.z(), footPosVelAtMiddle2.z());

    // Ending single support pose
    traj.get("trunk_pos_x").addPoint(endTrajectoryTime, targetTrunkPos().x(), 0.0);
    traj.get("trunk_pos_y").addPoint(endTrajectoryTime, targetTrunkPos().y(), 0.0);
    traj.get("trunk_pos_z").addPoint(endTrajectoryTime, targetTrunkPos().z(), 0.0);
    traj.get("trunk_axis_x").addPoint(endTrajectoryTime, targetTrunkAxis().x(), 0.0);
    traj.get("trunk_axis_y").addPoint(endTrajectoryTime, targetTrunkAxis().y(), 0.0);
    traj.get("trunk_axis_z").addPoint(endTrajectoryTime, targetTrunkAxis().z(), 0.0);
    traj.get("foot_pos_x").addPoint(endTrajectoryTime, targetFootPos().x(), 0.0);
    traj.get("foot_pos_y").addPoint(endTrajectoryTime, targetFootPos().y(), 0.0);
    traj.get("foot_pos_z").addPoint(endTrajectoryTime, targetFootPos().z(), 0.0);

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
    (void)joints;
    (void)dq;
    (void)ddq;

    Eigen::VectorXd tmpTorques = torques;
    tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;

    double cost = 0.0;
    cost += 0.01 * tmpTorques.norm();
    if (data.size() == 0)
    {
      data.push_back(0.0);
    }
    if (data[0] < tmpTorques.lpNorm<Eigen::Infinity>())
    {
      data[0] = tmpTorques.lpNorm<Eigen::Infinity>();
    }

    return cost;
  });
  generator.setEndScoreFunc([](const Eigen::VectorXd& params, const Leph::Trajectories& traj, double score,
                               std::vector<double>& data, bool verbose) -> double {
    (void)params;
    (void)traj;
    (void)score;
    (void)verbose;
    return data[0];
  });

  // Display initial trajectory
  TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

  // Target filename
  std::string filename = "/tmp/trajLegLift_" + Leph::currentDate() + ".splines";
  // Run the CMA-ES optimization
  generator.runOptimization(1000, 5, filename, 20, -1.0);

  // Display found trajectory
  TrajectoriesDisplay(generator.bestTrajectories());
}

void generateKick()
{
  double KickPosX = 0.01;
  double KickPosY = -0.11;
  double KickPosZ = 0.07;

  // Initialize the generator
  Leph::TrajectoryGeneration generator(Leph::SigmabanModel);

  // Initial trajectory parameters
  generator.setInitialParameters([&KickPosX, &KickPosY, &KickPosZ]() -> Eigen::VectorXd {
    Eigen::VectorXd params(51);
    // Contact time ratio
    params(0) = 0.5;
    // Contact trunk pos
    params(1) = targetTrunkPos().x();
    params(2) = targetTrunkPos().y();
    // Contact trunk vel
    params(3) = 0.0;
    params(4) = 0.0;
    // Contact trunk axis
    params(5) = targetTrunkAxis().x();
    params(6) = targetTrunkAxis().y();
    params(7) = targetTrunkAxis().z();
    // Contact trunk vel
    params(8) = 0.0;
    params(9) = 0.0;
    params(10) = 0.0;

    // Middle 1 trunk pos
    params(11) = targetTrunkPos().x() - 0.01;
    params(12) = targetTrunkPos().y();
    // Middle 1 trunk axis
    params(13) = targetTrunkAxis().x();
    params(14) = targetTrunkAxis().y();
    params(15) = targetTrunkAxis().z();
    // Middle 1 foot pos
    params(16) = KickPosX + 0.01;
    params(17) = KickPosZ;
    // Middle 1 foot vel
    params(18) = 0.0;
    params(19) = 0.0;

    // Middle 2 trunk pos
    params(20) = targetTrunkPos().x() + 0.01;
    params(21) = targetTrunkPos().y();
    // Middle 2 trunk vel
    params(22) = 0.0;
    params(23) = 0.0;
    // Middle 2 trunk axis
    params(24) = targetTrunkAxis().x();
    params(25) = targetTrunkAxis().y();
    params(26) = targetTrunkAxis().z();
    // Middle 2 trunk vel
    params(27) = 0.0;
    params(28) = 0.0;
    params(29) = 0.0;
    // Middle 2 foot pos
    params(30) = KickPosX - 0.01;
    params(31) = KickPosZ;
    // Middle 2 foot vel
    params(32) = 0.0;
    params(33) = 0.0;

    // Contact/Middle1/Middle2 accelerations
    params(34) = 0.0;
    params(35) = 0.0;
    params(36) = 0.0;
    params(37) = 0.0;
    params(38) = 0.0;
    params(39) = 0.0;
    params(40) = 0.0;
    params(41) = 0.0;
    params(42) = 0.0;
    params(43) = 0.0;
    params(44) = 0.0;
    params(45) = 0.0;
    params(46) = 0.0;
    params(47) = 0.0;
    params(48) = 0.0;

    // Middle1/Middle2 time ratio
    params(49) = 0.35;
    params(50) = 0.6;

    return params;
  }());

  // Set Trajectory generation function
  generator.setTrajectoryGenerationFunc(
      [&KickPosX, &KickPosY, &KickPosZ](const Eigen::VectorXd& params) -> Leph::Trajectories {
        double KickSpeed = 1.0;
        double endTrajectoryTime = 3.0;
        double ratioContactTime = params(0);

        double contactTime = ratioContactTime * endTrajectoryTime;
        Eigen::Vector3d trunkPosAtContact(params(1), params(2), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAtContact(params(3), params(4), 0.0);
        Eigen::Vector3d trunkAngleAtContact = params.segment<3>(5);
        Eigen::Vector3d trunkAngleVelAtContact = params.segment<3>(8);
        Eigen::Vector3d footPosAtContact(KickPosX, KickPosY, KickPosZ);
        Eigen::Vector3d footPosVelAtContact(KickSpeed, 0.0, 0.0);

        double middle1Time = params(49) * endTrajectoryTime;
        Eigen::Vector3d trunkPosAtMiddle1(params(11), params(12), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d trunkAngleAtMiddle1 = params.segment<3>(13);
        Eigen::Vector3d trunkAngleVelAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d footPosAtMiddle1(params(16), KickPosY, params(17));
        Eigen::Vector3d footPosVelAtMiddle1(params(18), 0.0, params(19));

        double middle2Time = params(50) * endTrajectoryTime;
        Eigen::Vector3d trunkPosAtMiddle2(params(20), params(21), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAtMiddle2(params(22), params(23), 0.0);
        Eigen::Vector3d trunkAngleAtMiddle2 = params.segment<3>(24);
        Eigen::Vector3d trunkAngleVelAtMiddle2 = params.segment<3>(27);
        Eigen::Vector3d footPosAtMiddle2(params(30), KickPosY, params(31));
        Eigen::Vector3d footPosVelAtMiddle2(params(32), 0.0, params(33));

        Eigen::Vector3d trunkPosAccAtContact(params(34), params(35), 0.0);
        Eigen::Vector3d trunkAngleAccAtContact(params(36), params(37), params(38));
        Eigen::Vector3d footPosAccAtContact(params(39), 0.0, 0.0);
        Eigen::Vector3d trunkPosAccAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d trunkAngleAccAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d footPosAccAtMiddle1(params(40), 0.0, params(41));
        Eigen::Vector3d trunkPosAccAtMiddle2(params(42), params(43), 0.0);
        Eigen::Vector3d trunkAngleAccAtMiddle2(params(44), params(45), params(46));
        Eigen::Vector3d footPosAccAtMiddle2(params(47), 0.0, params(48));

        // Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        // Support phase (single support for kick)
        traj.get("is_double_support").addPoint(0.0, 0.0, 0.0);
        traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);
        // Support foot
        traj.get("is_left_support_foot").addPoint(0.0, 1.0);
        traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0);

        // Starting double support pose
        traj.get("trunk_pos_x").addPoint(0.0, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(0.0, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(0.0, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(0.0, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(0.0, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(0.0, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(0.0, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(0.0, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(0.0, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);

        // Pre Kick middle
        traj.get("trunk_pos_x")
            .addPoint(middle1Time, trunkPosAtMiddle1.x(), trunkPosVelAtMiddle1.x(), trunkPosAccAtMiddle1.x());
        traj.get("trunk_pos_y")
            .addPoint(middle1Time, trunkPosAtMiddle1.y(), trunkPosVelAtMiddle1.y(), trunkPosAccAtMiddle1.y());
        traj.get("trunk_pos_z")
            .addPoint(middle1Time, trunkPosAtMiddle1.z(), trunkPosVelAtMiddle1.z(), trunkPosAccAtMiddle1.z());
        traj.get("trunk_axis_x")
            .addPoint(middle1Time, trunkAngleAtMiddle1.x(), trunkAngleVelAtMiddle1.x(), trunkAngleAccAtMiddle1.x());
        traj.get("trunk_axis_y")
            .addPoint(middle1Time, trunkAngleAtMiddle1.y(), trunkAngleVelAtMiddle1.y(), trunkAngleAccAtMiddle1.y());
        traj.get("trunk_axis_z")
            .addPoint(middle1Time, trunkAngleAtMiddle1.z(), trunkAngleVelAtMiddle1.z(), trunkAngleAccAtMiddle1.z());
        traj.get("foot_pos_x")
            .addPoint(middle1Time, footPosAtMiddle1.x(), footPosVelAtMiddle1.x(), footPosAccAtMiddle1.x());
        traj.get("foot_pos_y")
            .addPoint(middle1Time, footPosAtMiddle1.y(), footPosVelAtMiddle1.y(), footPosAccAtMiddle1.y());
        traj.get("foot_pos_z")
            .addPoint(middle1Time, footPosAtMiddle1.z(), footPosVelAtMiddle1.z(), footPosAccAtMiddle1.z());
        traj.get("foot_axis_x").addPoint(middle1Time, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(middle1Time, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(middle1Time, 0.0, 0.0);

        // Kick contact
        traj.get("trunk_pos_x")
            .addPoint(contactTime, trunkPosAtContact.x(), trunkPosVelAtContact.x(), trunkPosAccAtContact.x());
        traj.get("trunk_pos_y")
            .addPoint(contactTime, trunkPosAtContact.y(), trunkPosVelAtContact.y(), trunkPosAccAtContact.y());
        traj.get("trunk_pos_z")
            .addPoint(contactTime, trunkPosAtContact.z(), trunkPosVelAtContact.z(), trunkPosAccAtContact.z());
        traj.get("trunk_axis_x")
            .addPoint(contactTime, trunkAngleAtContact.x(), trunkAngleVelAtContact.x(), trunkAngleAccAtContact.x());
        traj.get("trunk_axis_y")
            .addPoint(contactTime, trunkAngleAtContact.y(), trunkAngleVelAtContact.y(), trunkAngleAccAtContact.y());
        traj.get("trunk_axis_z")
            .addPoint(contactTime, trunkAngleAtContact.z(), trunkAngleVelAtContact.z(), trunkAngleAccAtContact.z());
        traj.get("foot_pos_x")
            .addPoint(contactTime, footPosAtContact.x(), footPosVelAtContact.x(), footPosAccAtContact.x());
        traj.get("foot_pos_y")
            .addPoint(contactTime, footPosAtContact.y(), footPosVelAtContact.y(), footPosAccAtContact.y());
        traj.get("foot_pos_z")
            .addPoint(contactTime, footPosAtContact.z(), footPosVelAtContact.z(), footPosAccAtContact.z());
        traj.get("foot_axis_x").addPoint(contactTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(contactTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(contactTime, 0.0, 0.0);

        // Post Kick middle
        traj.get("trunk_pos_x")
            .addPoint(middle2Time, trunkPosAtMiddle2.x(), trunkPosVelAtMiddle2.x(), trunkPosAccAtMiddle2.x());
        traj.get("trunk_pos_y")
            .addPoint(middle2Time, trunkPosAtMiddle2.y(), trunkPosVelAtMiddle2.y(), trunkPosAccAtMiddle2.y());
        traj.get("trunk_pos_z")
            .addPoint(middle2Time, trunkPosAtMiddle2.z(), trunkPosVelAtMiddle2.z(), trunkPosAccAtMiddle2.z());
        traj.get("trunk_axis_x")
            .addPoint(middle2Time, trunkAngleAtMiddle2.x(), trunkAngleVelAtMiddle2.x(), trunkAngleAccAtMiddle2.x());
        traj.get("trunk_axis_y")
            .addPoint(middle2Time, trunkAngleAtMiddle2.y(), trunkAngleVelAtMiddle2.y(), trunkAngleAccAtMiddle2.y());
        traj.get("trunk_axis_z")
            .addPoint(middle2Time, trunkAngleAtMiddle2.z(), trunkAngleVelAtMiddle2.z(), trunkAngleAccAtMiddle2.z());
        traj.get("foot_pos_x")
            .addPoint(middle2Time, footPosAtMiddle2.x(), footPosVelAtMiddle2.x(), footPosAccAtMiddle2.x());
        traj.get("foot_pos_y")
            .addPoint(middle2Time, footPosAtMiddle2.y(), footPosVelAtMiddle2.y(), footPosAccAtMiddle2.y());
        traj.get("foot_pos_z")
            .addPoint(middle2Time, footPosAtMiddle2.z(), footPosVelAtMiddle2.z(), footPosAccAtMiddle2.z());
        traj.get("foot_axis_x").addPoint(middle2Time, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(middle2Time, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(middle2Time, 0.0, 0.0);

        // Ending single support pose
        traj.get("trunk_pos_x").addPoint(endTrajectoryTime, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(endTrajectoryTime, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(endTrajectoryTime, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(endTrajectoryTime, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(endTrajectoryTime, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(endTrajectoryTime, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(endTrajectoryTime, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(endTrajectoryTime, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(endTrajectoryTime, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

        return traj;
      });

  // Set parameters bound function
  generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
    // Check support ratio bound
    if (params(0) <= 0.1)
    {
      return 1000.0 - 1000.0 * params(0);
    }
    if (params(0) >= 0.9)
    {
      return 1000.0 + 1000.0 * (params(0) - 1.0);
    }
    if (params(49) <= 0.1)
    {
      return 1000.0 - 1000.0 * params(49);
    }
    if (params(49) >= 0.9)
    {
      return 1000.0 + 1000.0 * (params(49) - 1.0);
    }
    if (params(50) <= 0.1)
    {
      return 1000.0 - 1000.0 * params(50);
    }
    if (params(50) >= 0.9)
    {
      return 1000.0 + 1000.0 * (params(50) - 1.0);
    }
    if (params(49) + 0.05 > params(0))
    {
      return 1000.0 - 1000.0 * (params(0) - params(49) - 0.05);
    }
    if (params(0) + 0.05 > params(50))
    {
      return 1000.0 - 1000.0 * (params(50) - params(0) - 0.05);
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
    (void)isDoubleSupport;
    (void)supportFoot;

    double cost = 0.0;

    // Init data
    if (data.size() == 0)
    {
      data.push_back(0.0);
      data.push_back(0.0);
      data.push_back(0.0);
    }

    // ZMP
    Eigen::Vector3d zmp = model.zeroMomentPointSingleSupport("origin", torques);
    zmp.z() = 0.0;
    cost += zmp.norm();
    // Max ZMP
    if (data[0] < zmp.lpNorm<Eigen::Infinity>())
    {
      data[0] = zmp.lpNorm<Eigen::Infinity>();
    }

    // Support torque yaw
    double torqueSupportYaw = fabs(torques(model.get().getDOFIndex("base_yaw")));
    // Max support yaw
    if (data[2] < torqueSupportYaw)
    {
      data[2] = torqueSupportYaw;
    }

    // Torques
    Eigen::VectorXd tmpTorques = torques;
    tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;
    cost += 0.01 * tmpTorques.norm();

    // Voltage
    for (const std::string& name : Leph::NamesDOF)
    {
      size_t index = model.get().getDOFIndex(name);
      double volt = joints.at(name).computeElectricTension(dq(index), ddq(index), torques(index));
      // Maximum voltage
      if (data[1] < volt)
      {
        data[1] = volt;
      }
    }

    return cost;
  });
  generator.setEndScoreFunc([](const Eigen::VectorXd& params, const Leph::Trajectories& traj, double score,
                               std::vector<double>& data, bool verbose) -> double {
    (void)params;
    (void)traj;
    if (verbose)
    {
      std::cout << "SumTorque=" << score << " MaxZMP=" << data[0] << " MaxVolt=" << data[1]
                << " MaxTorqueYaw=" << data[2] << std::endl;
    }
    double cost = 0.0;
    if (data[2] > 0.2)
    {
      cost += 10.0 + 10.0 * data[2];
      if (verbose)
      {
        std::cout << "TorqueYawBound=" << cost << std::endl;
      }
    }
    if (fabs(data[1]) > 0.75 * 12.0)
    {
      cost += 10.0 + 10.0 * data[1];
      if (verbose)
      {
        std::cout << "VoltBound=" << cost << std::endl;
      }
    }
    else
    {
      cost += 150.0 * data[0] + 0.2 * data[1] + 2.0 * data[2];
      if (verbose)
      {
        std::cout << "ZMPCost=" << 150.0 * data[0] << " VoltCost=" << 0.2 * data[1]
                  << " TorqueYawCost=" << 2.0 * data[2] << std::endl;
      }
    }
    if (verbose)
    {
      std::cout << "--> "
                << "IterationCost=" << score << " EndCost=" << cost << " Total=" << score + cost << std::endl;
    }
    return cost;
  });

  // Display initial trajectory
  TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

  // Target filename
  std::string filename = "/tmp/trajKick_" + Leph::currentDate();
  // Run the CMA-ES optimization
  generator.runOptimization(2000, 5, filename, 20, -1.0);

  // Display found trajectory
  TrajectoriesDisplay(generator.bestTrajectories());
}

void generateStaticSingleSupport()
{
  // Initialize the generator
  Leph::TrajectoryGeneration generator(Leph::SigmabanModel);

  // Initial trajectory parameters
  generator.setInitialParameters([]() -> Eigen::VectorXd {
    Eigen::VectorXd params(9);
    params(0) = initTrunkPos().x();
    params(1) = initTrunkPos().y();
    params(2) = initTrunkPos().z();
    params(3) = initTrunkAxis().x();
    params(4) = initTrunkAxis().y();
    params(5) = initTrunkAxis().z();
    params(6) = initFootPos().x();
    params(7) = initFootPos().y();
    params(8) = initFootPos().z();

    return params;
  }());

  // Set Trajectory generation function
  generator.setTrajectoryGenerationFunc([](const Eigen::VectorXd& params) -> Leph::Trajectories {
    double endTrajectoryTime = 0.1;
    Eigen::Vector3d trunkPos = params.segment<3>(0);
    Eigen::Vector3d trunkAxis = params.segment<3>(3);
    Eigen::Vector3d footPos = params.segment<3>(6);

    // Initialize state splines
    Leph::Trajectories traj = Leph::TrajectoriesInit();

    // Support phase (single support for kick)
    traj.get("is_double_support").addPoint(0.0, 0.0, 0.0);
    traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);
    // Support foot
    traj.get("is_left_support_foot").addPoint(0.0, 1.0);
    traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0);

    traj.get("trunk_pos_x").addPoint(0.0, trunkPos.x(), 0.0);
    traj.get("trunk_pos_y").addPoint(0.0, trunkPos.y(), 0.0);
    traj.get("trunk_pos_z").addPoint(0.0, trunkPos.z(), 0.0);
    traj.get("trunk_axis_x").addPoint(0.0, trunkAxis.x(), 0.0);
    traj.get("trunk_axis_y").addPoint(0.0, trunkAxis.y(), 0.0);
    traj.get("trunk_axis_z").addPoint(0.0, trunkAxis.z(), 0.0);
    traj.get("foot_pos_x").addPoint(0.0, footPos.x(), 0.0);
    traj.get("foot_pos_y").addPoint(0.0, footPos.y(), 0.0);
    traj.get("foot_pos_z").addPoint(0.0, footPos.z(), 0.0);
    traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
    traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);

    traj.get("trunk_pos_x").addPoint(endTrajectoryTime, trunkPos.x(), 0.0);
    traj.get("trunk_pos_y").addPoint(endTrajectoryTime, trunkPos.y(), 0.0);
    traj.get("trunk_pos_z").addPoint(endTrajectoryTime, trunkPos.z(), 0.0);
    traj.get("trunk_axis_x").addPoint(endTrajectoryTime, trunkAxis.x(), 0.0);
    traj.get("trunk_axis_y").addPoint(endTrajectoryTime, trunkAxis.y(), 0.0);
    traj.get("trunk_axis_z").addPoint(endTrajectoryTime, trunkAxis.z(), 0.0);
    traj.get("foot_pos_x").addPoint(endTrajectoryTime, footPos.x(), 0.0);
    traj.get("foot_pos_y").addPoint(endTrajectoryTime, footPos.y(), 0.0);
    traj.get("foot_pos_z").addPoint(endTrajectoryTime, footPos.z(), 0.0);
    traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
    traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
    traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

    return traj;
  });

  // Set parameters bound function
  generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
    (void)params;
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
    (void)joints;
    (void)dq;
    (void)ddq;
    (void)data;

    Eigen::VectorXd tmpTorques = torques;
    tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;

    double cost = 0.0;
    cost += 0.01 * tmpTorques.norm();
    return cost;
  });
  generator.setEndScoreFunc([](const Eigen::VectorXd& params, const Leph::Trajectories& traj, double score,
                               std::vector<double>& data, bool verbose) -> double {
    (void)params;
    (void)data;
    (void)traj;
    return 0.0;
  });

  // Display initial trajectory
  TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

  // Target filename
  std::string filename = "/tmp/trajStatic_" + Leph::currentDate() + ".splines";
  // Run the CMA-ES optimization
  generator.runOptimization(1000, 5, filename, 20, -1.0);

  // Display found trajectory
  TrajectoriesDisplay(generator.bestTrajectories());
}

/*
void generateRecovery()
{
    //Initialize the generator
    Leph::TrajectoryGeneration generator(Leph::SigmabanModel);

    //Initial trajectory parameters
    generator.setInitialParameters([]() -> Eigen::VectorXd {
        Eigen::VectorXd params(48);
        //Time ratio
        params(0) = 0.2;
        params(1) = 0.5;
        params(2) = 0.8;

        //Pos time 1
        params(3) = targetTrunkPos().x();
        params(4) = targetTrunkPos().y();
        params(9) = targetTrunkAxis().x();
        params(10) = targetTrunkAxis().y();
        params(11) = targetTrunkAxis().z();
        //Vel/Acc time 1
        params.segment<2>(5).setZero();
        params.segment<2>(7).setZero();
        params.segment<3>(12).setZero();
        params.segment<3>(15).setZero();

        //Pos time 2
        params(18) = targetTrunkPos().x();
        params(19) = targetTrunkPos().y();
        params(24) = targetTrunkAxis().x();
        params(25) = targetTrunkAxis().y();
        params(26) = targetTrunkAxis().z();
        //Vel/Acc time 2
        params.segment<2>(20).setZero();
        params.segment<2>(22).setZero();
        params.segment<3>(27).setZero();
        params.segment<3>(30).setZero();

        //Pos time 3
        params(33) = targetTrunkPos().x();
        params(34) = targetTrunkPos().y();
        params(39) = targetTrunkAxis().x();
        params(40) = targetTrunkAxis().y();
        params(41) = targetTrunkAxis().z();
        //Vel/Acc time 3
        params.segment<2>(35).setZero();
        params.segment<2>(37).setZero();
        params.segment<3>(42).setZero();
        params.segment<3>(45).setZero();

        return params;
    }());

    double endTrajectoryTime = 2.0;
    //Set Trajectory generation function
    generator.setTrajectoryGenerationFunc([&endTrajectoryTime](const Eigen::VectorXd& params) -> Leph::Trajectories {
        double time1 = params(0) * endTrajectoryTime;
        double time2 = params(1) * endTrajectoryTime;
        double time3 = params(2) * endTrajectoryTime;

        Eigen::Vector3d trunkPosAt1(params(3), params(4), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAt1(params(5), params(6), 0.0);
        Eigen::Vector3d trunkPosAccAt1(params(7), params(8), 0.0);
        Eigen::Vector3d trunkAxisAt1 = params.segment<3>(9);
        Eigen::Vector3d trunkAxisVelAt1 = params.segment<3>(12);
        Eigen::Vector3d trunkAxisAccAt1 = params.segment<3>(15);

        Eigen::Vector3d trunkPosAt2(params(18), params(19), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAt2(params(20), params(21), 0.0);
        Eigen::Vector3d trunkPosAccAt2(params(22), params(23), 0.0);
        Eigen::Vector3d trunkAxisAt2 = params.segment<3>(24);
        Eigen::Vector3d trunkAxisVelAt2 = params.segment<3>(27);
        Eigen::Vector3d trunkAxisAccAt2 = params.segment<3>(30);

        Eigen::Vector3d trunkPosAt3(params(33), params(34), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAt3(params(35), params(36), 0.0);
        Eigen::Vector3d trunkPosAccAt3(params(37), params(38), 0.0);
        Eigen::Vector3d trunkAxisAt3 = params.segment<3>(39);
        Eigen::Vector3d trunkAxisVelAt3 = params.segment<3>(42);
        Eigen::Vector3d trunkAxisAccAt3 = params.segment<3>(45);

        //Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        //Support phase (single support for kick)
        traj.get("is_double_support").addPoint(0.0, 0.0, 0.0);
        traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);
        //Support foot
        traj.get("is_left_support_foot").addPoint(0.0, 1.0);
        traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0);

        //Starting position
        traj.get("trunk_pos_x").addPoint(0.0, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(0.0, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(0.0, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(0.0, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(0.0, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(0.0, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(0.0, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(0.0, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(0.0, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);

        //Time 1
        traj.get("trunk_pos_x").addPoint(time1, trunkPosAt1.x(), trunkPosVelAt1.x(), trunkPosAccAt1.x());
        traj.get("trunk_pos_y").addPoint(time1, trunkPosAt1.y(), trunkPosVelAt1.y(), trunkPosAccAt1.y());
        traj.get("trunk_pos_z").addPoint(time1, trunkPosAt1.z(), trunkPosVelAt1.z(), trunkPosAccAt1.z());
        traj.get("trunk_axis_x").addPoint(time1, trunkAxisAt1.x(), trunkAxisVelAt1.x(), trunkAxisAccAt1.x());
        traj.get("trunk_axis_y").addPoint(time1, trunkAxisAt1.y(), trunkAxisVelAt1.y(), trunkAxisAccAt1.y());
        traj.get("trunk_axis_z").addPoint(time1, trunkAxisAt1.z(), trunkAxisVelAt1.z(), trunkAxisAccAt1.z());
        //Time 2
        traj.get("trunk_pos_x").addPoint(time2, trunkPosAt2.x(), trunkPosVelAt2.x(), trunkPosAccAt2.x());
        traj.get("trunk_pos_y").addPoint(time2, trunkPosAt2.y(), trunkPosVelAt2.y(), trunkPosAccAt2.y());
        traj.get("trunk_pos_z").addPoint(time2, trunkPosAt2.z(), trunkPosVelAt2.z(), trunkPosAccAt2.z());
        traj.get("trunk_axis_x").addPoint(time2, trunkAxisAt2.x(), trunkAxisVelAt2.x(), trunkAxisAccAt2.x());
        traj.get("trunk_axis_y").addPoint(time2, trunkAxisAt2.y(), trunkAxisVelAt2.y(), trunkAxisAccAt2.y());
        traj.get("trunk_axis_z").addPoint(time2, trunkAxisAt2.z(), trunkAxisVelAt2.z(), trunkAxisAccAt2.z());
        //Time 3
        traj.get("trunk_pos_x").addPoint(time3, trunkPosAt3.x(), trunkPosVelAt3.x(), trunkPosAccAt3.x());
        traj.get("trunk_pos_y").addPoint(time3, trunkPosAt3.y(), trunkPosVelAt3.y(), trunkPosAccAt3.y());
        traj.get("trunk_pos_z").addPoint(time3, trunkPosAt3.z(), trunkPosVelAt3.z(), trunkPosAccAt3.z());
        traj.get("trunk_axis_x").addPoint(time3, trunkAxisAt3.x(), trunkAxisVelAt3.x(), trunkAxisAccAt3.x());
        traj.get("trunk_axis_y").addPoint(time3, trunkAxisAt3.y(), trunkAxisVelAt3.y(), trunkAxisAccAt3.y());
        traj.get("trunk_axis_z").addPoint(time3, trunkAxisAt3.z(), trunkAxisVelAt3.z(), trunkAxisAccAt3.z());

        //Final position
        traj.get("trunk_pos_x").addPoint(endTrajectoryTime, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(endTrajectoryTime, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(endTrajectoryTime, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(endTrajectoryTime, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(endTrajectoryTime, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(endTrajectoryTime, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(endTrajectoryTime, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(endTrajectoryTime, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(endTrajectoryTime, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

        //Prediction of base_pitch DOF
        Leph::Plot plot;
        Leph::FittedSpline fitted;
        double basePitch1 = 0.15;
        double basePitch2 = 0.13;
        fitted.addPoint(traj.min(), 0.13);
        fitted.addPoint(traj.min()+0.02, 0.15);
        for (double t=traj.min()+0.04;t<=traj.max();t+=0.02) {
            Eigen::VectorXd in(19);
            in <<
                1.0,
                basePitch1,
                pow(basePitch1, 2),
                pow(basePitch1, 3),
                basePitch2,
                pow(basePitch2, 2),
                pow(basePitch2, 3),
                traj.get("trunk_axis_y").pos(t),
                pow(traj.get("trunk_axis_y").pos(t), 2),
                pow(traj.get("trunk_axis_y").pos(t), 3),
                traj.get("trunk_axis_y").pos(t-0.04),
                pow(traj.get("trunk_axis_y").pos(t-0.04), 2),
                pow(traj.get("trunk_axis_y").pos(t-0.04), 3),
                traj.get("trunk_pos_x").pos(t),
                pow(traj.get("trunk_pos_x").pos(t), 2),
                pow(traj.get("trunk_pos_x").pos(t), 3),
                traj.get("trunk_pos_x").pos(t-0.04),
                pow(traj.get("trunk_pos_x").pos(t-0.04), 2),
                pow(traj.get("trunk_pos_x").pos(t-0.04), 3);
            Eigen::VectorXd params(19);
            //params << 0.0103546211334937,    1.74794674232662, -0.0199965144590223,   -1.47061678542555,
-0.725820337320682,  -0.443239683432601,    2.73392480277782,   0.256862313765961,   -1.33153138729058,
3.55420789543342,  -0.283590499587426,   0.573616200027463,   -4.38768196123613,  0.0305010006062203, 22.8659275289408,
23.2178130052898,    1.06654758979916,    7.50814284189734,    18.0400387045032;

            params << 0.171175921540038,  -2.04938242995119,   26.4791690980276,  -133.429826772291, -1.97550591755917,
33.7710347540175,  -151.993181081481, -0.847598940928787,  -1.36185904648208,   3.57343177625409,  0.826883316896905,
1.47720437598528,  -4.18264295969538,  -1.50553611404296,  -6.87438200245366,  -365.137332830405,   2.26947070272837,
6.24171723202638,   286.151684252741; double value = in.dot(params); if (value > 1.0) value = 1.0; if (value < -1.0)
value = -1.0; basePitch2 = basePitch1; basePitch1 = value; fitted.addPoint(t, value); plot.add(Leph::VectorLabel("t", t,
"value", value));
        }
        fitted.fittingSmooth(6);
        traj.add("base_pitch");
        traj.get("base_pitch").copyData(fitted);
        for (double t=traj.min();t<=traj.max();t+=0.01) {
            plot.add(Leph::VectorLabel("t", t, "fitted", traj.get("base_pitch").pos(t)));
        }
        //plot.plot("t", "all").render();

        return traj;
    });

    //Set parameters bound function
    generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
        if (params(0) <= 0.1) {
            return 1000.0 - 1000.0*(params(0) - 0.1);
        }
        if (params(0) >= 0.9) {
            return 1000.0 + 1000.0*(params(0) - 0.9);
        }
        if (params(1) <= 0.1) {
            return 1000.0 - 1000.0*(params(1) - 0.1);
        }
        if (params(1) >= 0.9) {
            return 1000.0 + 1000.0*(params(1) - 0.9);
        }
        if (params(2) <= 0.1) {
            return 1000.0 - 1000.0*(params(2) - 0.1);
        }
        if (params(2) >= 0.9) {
            return 1000.0 + 1000.0*(params(2) - 0.9);
        }
        if (params(0) + 0.1 > params(1)) {
            return 1000.0 - 1000.0*(params(1)-params(0)-0.1);
        }
        if (params(1) + 0.1 > params(2)) {
            return 1000.0 - 1000.0*(params(2)-params(1)-0.1);
        }
        return 0.0;
    });
    //Set default Cartesian state and Joint DOF bounds
    generator.setCheckStateFunc(Leph::DefaultCheckState);
    generator.setCheckDOFFunc(Leph::DefaultCheckDOF);

    //Set trajectory scoring function
    generator.setScoreFunc([](
        double t,
        Leph::HumanoidFixedModel& model,
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        bool isDoubleSupport,
        Leph::HumanoidFixedModel::SupportFoot supportFoot,
        std::vector<double>& data) -> double
    {
        (void)data;

        Eigen::VectorXd tmpTorques = torques;
        tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
        //tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;

        double cost = 0.0;
        cost += 0.01*tmpTorques.norm();
        Eigen::Vector3d zmp = model.zeroMomentPointSingleSupport("origin", torques);
        cost += fabs(zmp.y());
        cost += fabs(zmp.x());

        return cost;
    });
    generator.setEndScoreFunc([](
        const Eigen::VectorXd& params,
        const Leph::Trajectories& traj,
        std::vector<double>& data) -> double {
        (void)params;
        (void)traj;
        (void)data;
        return 0.0;
    });

    //Display initial trajectory
    TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

    //Target filename
    std::string filename = "/tmp/trajRecovery_" + Leph::currentDate() + ".splines";
    //Run the CMA-ES optimization
    generator.runOptimization(2000, 5, filename, 20, -1.0);

    //Display found trajectory
    TrajectoriesDisplay(generator.bestTrajectories());
}
*/

void generateWalk()
{
  // Initialize the generator
  Leph::TrajectoryGeneration generator(Leph::SigmabanModel);

  // Initial trajectory parameters
  generator.setInitialParameters([]() -> Eigen::VectorXd {
    // Parameters initialization
    Eigen::VectorXd params = Eigen::VectorXd::Zero(61);

    // Double support ratio
    params(0) = 1.5;
    // params(0) = 0.3;

    // Trunk pos double support begin
    params(1) = -0.01;
    params(2) = -0.03;
    params(3) = 0.24;
    // Trunk pos velocity x/y
    params(4) = 0.2;
    params(5) = -0.2;

    // Trunk pos double support end
    params(19) = 0.01;
    params(20) = -0.07;
    params(21) = 0.24;
    // Trunk pos velocity x/y
    params(22) = 0.2;
    params(23) = -0.2;

    // Foot double support lateral
    params(37) = -0.115;

    // Trunk position single support
    params(38) = -0.04;
    params(39) = -0.01;
    params(40) = 0.24;
    // Trunk position x vel
    params(41) = 0.1;
    // Trunk axis x
    // params(46) = -0.1;
    // Foot position x/y
    params(54) = 0.0;
    params(55) = -0.115;
    // Foot velocity x
    params(56) = 0.3;

    return params;
  }());

  // Set Trajectory generation function
  generator.setTrajectoryGenerationFunc([](const Eigen::VectorXd& params) -> Leph::Trajectories {
    // Walk configuration
    double footHeight = 0.035;
    double footStep = 0.04;
    // double cycleLength = 1.0;
    double cycleLength = params(0);

    // Double support time ratio
    double doubleSupportRatio = 0.0;
    // double doubleSupportRatio = params(0);
    double timeDSLength = doubleSupportRatio * cycleLength / 2.0;
    double timeSSLength = (1.0 - doubleSupportRatio) * cycleLength / 2.0;

    // Time ratio
    double timeDSLeftBegin1 = 0.0;
    double timeDSLeftEnd = timeDSLeftBegin1 + timeDSLength;
    double timeSSRightApex = timeDSLeftEnd + timeSSLength / 2.0;
    double timeDSRightBegin = timeSSRightApex + timeSSLength / 2.0;
    double timeDSRightEnd = timeDSRightBegin + timeDSLength;
    double timeSSLeftApex = timeDSRightEnd + timeSSLength / 2.0;
    double timeDSLeftBegin2 = timeSSLeftApex + timeSSLength / 2.0;

    // Double support target state
    Eigen::Vector3d trunkPos_DSBegin(params(1), params(2), params(3));
    Eigen::Vector3d trunkPosVel_DSBegin(params(4), params(5), params(6));
    Eigen::Vector3d trunkPosAcc_DSBegin(params(7), params(8), params(9));
    Eigen::Vector3d trunkAxis_DSBegin(params(10), params(11), params(12));
    Eigen::Vector3d trunkAxisVel_DSBegin(params(13), params(14), params(15));
    Eigen::Vector3d trunkAxisAcc_DSBegin(params(16), params(17), params(18));
    Eigen::Vector3d trunkPos_DSEnd(params(19), params(20), params(21));
    Eigen::Vector3d trunkPosVel_DSEnd(params(22), params(23), params(24));
    Eigen::Vector3d trunkPosAcc_DSEnd(params(25), params(26), params(27));
    Eigen::Vector3d trunkAxis_DSEnd(params(28), params(29), params(30));
    Eigen::Vector3d trunkAxisVel_DSEnd(params(31), params(32), params(33));
    Eigen::Vector3d trunkAxisAcc_DSEnd(params(34), params(35), params(36));
    Eigen::Vector3d footPos_DS(footStep, -0.12, 0.0);
    // Eigen::Vector3d footPos_DS(footStep, params(37), 0.0); TODO TODO

    // Single support target state
    Eigen::Vector3d trunkPos_SS(params(38), params(39), params(40));
    Eigen::Vector3d trunkPosVel_SS(params(41), 0.0, params(42));
    Eigen::Vector3d trunkPosAcc_SS(params(43), params(44), params(45));
    Eigen::Vector3d trunkAxis_SS(params(46), params(47), params(48));
    Eigen::Vector3d trunkAxisVel_SS(0.0, params(49), params(50));
    Eigen::Vector3d trunkAxisAcc_SS(params(51), params(52), params(53));
    Eigen::Vector3d footPos_SS(params(54), -0.12, footHeight);
    // Eigen::Vector3d footPos_SS(params(54), params(55), footHeight); TODO TODO
    Eigen::Vector3d footPosVel_SS(params(56), params(57), 0.0);
    Eigen::Vector3d footPosAcc_SS(params(58), params(59), params(60));

    // Initialize state splines
    Leph::Trajectories traj = Leph::TrajectoriesInit();

    // Support foot phase
    traj.get("is_double_support").addPoint(timeDSLeftBegin1, 1.0);
    traj.get("is_double_support").addPoint(timeDSLeftEnd, 1.0);
    traj.get("is_double_support").addPoint(timeDSLeftEnd, 0.0);
    traj.get("is_double_support").addPoint(timeDSRightBegin, 0.0);
    traj.get("is_double_support").addPoint(timeDSRightBegin, 1.0);
    traj.get("is_double_support").addPoint(timeDSRightEnd, 1.0);
    traj.get("is_double_support").addPoint(timeDSRightEnd, 0.0);
    traj.get("is_double_support").addPoint(timeDSLeftBegin2, 0.0);
    // Support foot
    traj.get("is_left_support_foot").addPoint(timeDSLeftBegin1, 1.0);
    traj.get("is_left_support_foot").addPoint(timeDSLeftEnd, 1.0);
    traj.get("is_left_support_foot").addPoint(timeDSLeftEnd, 0.0);
    traj.get("is_left_support_foot").addPoint(timeDSRightEnd, 0.0);
    traj.get("is_left_support_foot").addPoint(timeDSRightEnd, 1.0);
    traj.get("is_left_support_foot").addPoint(timeDSLeftBegin2, 1.0);

    // Foot orientation
    traj.get("foot_axis_x").addPoint(timeDSLeftBegin1, 0.0);
    traj.get("foot_axis_y").addPoint(timeDSLeftBegin1, 0.0);
    traj.get("foot_axis_z").addPoint(timeDSLeftBegin1, 0.0);
    traj.get("foot_axis_x").addPoint(timeDSLeftBegin2, 0.0);
    traj.get("foot_axis_y").addPoint(timeDSLeftBegin2, 0.0);
    traj.get("foot_axis_z").addPoint(timeDSLeftBegin2, 0.0);

    /*
    //Begin double support left 1
    traj.get("trunk_pos_x").addPoint(timeDSLeftBegin1, trunkPos_DSBegin.x(), trunkPosVel_DSBegin.x(),
    trunkPosAcc_DSBegin.x()); traj.get("trunk_pos_y").addPoint(timeDSLeftBegin1, trunkPos_DSBegin.y(),
    trunkPosVel_DSBegin.y(), trunkPosAcc_DSBegin.y()); traj.get("trunk_pos_z").addPoint(timeDSLeftBegin1,
    trunkPos_DSBegin.z(), trunkPosVel_DSBegin.z(), trunkPosAcc_DSBegin.z());
    traj.get("trunk_axis_x").addPoint(timeDSLeftBegin1, trunkAxis_DSBegin.x(), trunkAxisVel_DSBegin.x(),
    trunkAxisAcc_DSBegin.x()); traj.get("trunk_axis_y").addPoint(timeDSLeftBegin1, trunkAxis_DSBegin.y(),
    trunkAxisVel_DSBegin.y(), trunkAxisAcc_DSBegin.y()); traj.get("trunk_axis_z").addPoint(timeDSLeftBegin1,
    trunkAxis_DSBegin.z(), trunkAxisVel_DSBegin.z(), trunkAxisAcc_DSBegin.z());
    traj.get("foot_pos_x").addPoint(timeDSLeftBegin1, footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSLeftBegin1, footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSLeftBegin1, footPos_DS.z(), 0.0, 0.0);
    //End double support left
    traj.get("trunk_pos_x").addPoint(timeDSLeftEnd, trunkPos_DSEnd.x(), trunkPosVel_DSEnd.x(), trunkPosAcc_DSEnd.x());
    traj.get("trunk_pos_y").addPoint(timeDSLeftEnd, trunkPos_DSEnd.y(), trunkPosVel_DSEnd.y(), trunkPosAcc_DSEnd.y());
    traj.get("trunk_pos_z").addPoint(timeDSLeftEnd, trunkPos_DSEnd.z(), trunkPosVel_DSEnd.z(), trunkPosAcc_DSEnd.z());
    traj.get("trunk_axis_x").addPoint(timeDSLeftEnd, trunkAxis_DSEnd.x(), trunkAxisVel_DSEnd.x(),
    trunkAxisAcc_DSEnd.x()); traj.get("trunk_axis_y").addPoint(timeDSLeftEnd, trunkAxis_DSEnd.y(),
    trunkAxisVel_DSEnd.y(), trunkAxisAcc_DSEnd.y()); traj.get("trunk_axis_z").addPoint(timeDSLeftEnd,
    trunkAxis_DSEnd.z(), trunkAxisVel_DSEnd.z(), trunkAxisAcc_DSEnd.z()); traj.get("foot_pos_x").addPoint(timeDSLeftEnd,
    footPos_DS.x(), 0.0, 0.0); traj.get("foot_pos_y").addPoint(timeDSLeftEnd, footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSLeftEnd, footPos_DS.z(), 0.0, 0.0);
    */

    // Support foot swap
    traj.get("trunk_pos_x")
        .addPoint(timeDSLeftEnd, trunkPos_DSEnd.x() - footPos_DS.x(), trunkPosVel_DSEnd.x(), trunkPosAcc_DSEnd.x());
    traj.get("trunk_pos_y")
        .addPoint(timeDSLeftEnd, trunkPos_DSEnd.y() - footPos_DS.y(), trunkPosVel_DSEnd.y(), trunkPosAcc_DSEnd.y());
    traj.get("trunk_pos_z").addPoint(timeDSLeftEnd, trunkPos_DSEnd.z(), trunkPosVel_DSEnd.z(), trunkPosAcc_DSEnd.z());
    traj.get("trunk_axis_x")
        .addPoint(timeDSLeftEnd, trunkAxis_DSEnd.x(), trunkAxisVel_DSEnd.x(), trunkAxisAcc_DSEnd.x());
    traj.get("trunk_axis_y")
        .addPoint(timeDSLeftEnd, trunkAxis_DSEnd.y(), trunkAxisVel_DSEnd.y(), trunkAxisAcc_DSEnd.y());
    traj.get("trunk_axis_z")
        .addPoint(timeDSLeftEnd, trunkAxis_DSEnd.z(), trunkAxisVel_DSEnd.z(), trunkAxisAcc_DSEnd.z());
    traj.get("foot_pos_x").addPoint(timeDSLeftEnd, -footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSLeftEnd, -footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSLeftEnd, footPos_DS.z(), 0.0, 0.0);

    // Apex at right single support
    traj.get("trunk_pos_x").addPoint(timeSSRightApex, trunkPos_SS.x(), trunkPosVel_SS.x(), trunkPosAcc_SS.x());
    traj.get("trunk_pos_y").addPoint(timeSSRightApex, -trunkPos_SS.y(), -trunkPosVel_SS.y(), -trunkPosAcc_SS.y());
    traj.get("trunk_pos_z").addPoint(timeSSRightApex, trunkPos_SS.z(), trunkPosVel_SS.z(), trunkPosAcc_SS.z());
    traj.get("trunk_axis_x").addPoint(timeSSRightApex, -trunkAxis_SS.x(), -trunkAxisVel_SS.x(), -trunkAxisAcc_SS.x());
    traj.get("trunk_axis_y").addPoint(timeSSRightApex, trunkAxis_SS.y(), trunkAxisVel_SS.y(), trunkAxisAcc_SS.y());
    traj.get("trunk_axis_z").addPoint(timeSSRightApex, trunkAxis_SS.z(), trunkAxisVel_SS.z(), trunkAxisAcc_SS.z());
    traj.get("foot_pos_x").addPoint(timeSSRightApex, footPos_SS.x(), footPosVel_SS.x(), footPosAcc_SS.x());
    traj.get("foot_pos_y").addPoint(timeSSRightApex, -footPos_SS.y(), -footPosVel_SS.y(), -footPosAcc_SS.y());
    traj.get("foot_pos_z").addPoint(timeSSRightApex, footPos_SS.z(), footPosVel_SS.z(), footPosAcc_SS.z());

    // Begin double support right
    /*
    traj.get("trunk_pos_x").addPoint(timeDSRightBegin, trunkPos_DSBegin.x(), trunkPosVel_DSBegin.x(),
    trunkPosAcc_DSBegin.x()); traj.get("trunk_pos_y").addPoint(timeDSRightBegin, -trunkPos_DSBegin.y(),
    -trunkPosVel_DSBegin.y(), -trunkPosAcc_DSBegin.y()); traj.get("trunk_pos_z").addPoint(timeDSRightBegin,
    trunkPos_DSBegin.z(), trunkPosVel_DSBegin.z(), trunkPosAcc_DSBegin.z());
    traj.get("trunk_axis_x").addPoint(timeDSRightBegin, -trunkAxis_DSBegin.x(), -trunkAxisVel_DSBegin.x(),
    -trunkAxisAcc_DSBegin.x()); traj.get("trunk_axis_y").addPoint(timeDSRightBegin, trunkAxis_DSBegin.y(),
    trunkAxisVel_DSBegin.y(), trunkAxisAcc_DSBegin.y()); traj.get("trunk_axis_z").addPoint(timeDSRightBegin,
    trunkAxis_DSBegin.z(), trunkAxisVel_DSBegin.z(), trunkAxisAcc_DSBegin.z());
    traj.get("foot_pos_x").addPoint(timeDSRightBegin, footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSRightBegin, -footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSRightBegin, footPos_DS.z(), 0.0, 0.0);
    */
    traj.get("trunk_pos_x")
        .addPoint(timeDSRightBegin, trunkPos_DSEnd.x(), trunkPosVel_DSEnd.x(), trunkPosAcc_DSEnd.x());
    traj.get("trunk_pos_y")
        .addPoint(timeDSRightBegin, -trunkPos_DSEnd.y(), -trunkPosVel_DSEnd.y(), -trunkPosAcc_DSEnd.y());
    traj.get("trunk_pos_z")
        .addPoint(timeDSRightBegin, trunkPos_DSEnd.z(), trunkPosVel_DSEnd.z(), trunkPosAcc_DSEnd.z());
    traj.get("trunk_axis_x")
        .addPoint(timeDSRightBegin, -trunkAxis_DSEnd.x(), -trunkAxisVel_DSEnd.x(), -trunkAxisAcc_DSEnd.x());
    traj.get("trunk_axis_y")
        .addPoint(timeDSRightBegin, trunkAxis_DSEnd.y(), trunkAxisVel_DSEnd.y(), trunkAxisAcc_DSEnd.y());
    traj.get("trunk_axis_z")
        .addPoint(timeDSRightBegin, trunkAxis_DSEnd.z(), trunkAxisVel_DSEnd.z(), trunkAxisAcc_DSEnd.z());
    traj.get("foot_pos_x").addPoint(timeDSRightBegin, footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSRightBegin, -footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSRightBegin, footPos_DS.z(), 0.0, 0.0);

    // End double support right
    traj.get("trunk_pos_x").addPoint(timeDSRightEnd, trunkPos_DSEnd.x(), trunkPosVel_DSEnd.x(), trunkPosAcc_DSEnd.x());
    traj.get("trunk_pos_y")
        .addPoint(timeDSRightEnd, -trunkPos_DSEnd.y(), -trunkPosVel_DSEnd.y(), -trunkPosAcc_DSEnd.y());
    traj.get("trunk_pos_z").addPoint(timeDSRightEnd, trunkPos_DSEnd.z(), trunkPosVel_DSEnd.z(), trunkPosAcc_DSEnd.z());
    traj.get("trunk_axis_x")
        .addPoint(timeDSRightEnd, -trunkAxis_DSEnd.x(), -trunkAxisVel_DSEnd.x(), -trunkAxisAcc_DSEnd.x());
    traj.get("trunk_axis_y")
        .addPoint(timeDSRightEnd, trunkAxis_DSEnd.y(), trunkAxisVel_DSEnd.y(), trunkAxisAcc_DSEnd.y());
    traj.get("trunk_axis_z")
        .addPoint(timeDSRightEnd, trunkAxis_DSEnd.z(), trunkAxisVel_DSEnd.z(), trunkAxisAcc_DSEnd.z());
    traj.get("foot_pos_x").addPoint(timeDSRightEnd, footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSRightEnd, -footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSRightEnd, footPos_DS.z(), 0.0, 0.0);

    // Support foot swap
    traj.get("trunk_pos_x")
        .addPoint(timeDSRightEnd, trunkPos_DSEnd.x() - footPos_DS.x(), trunkPosVel_DSEnd.x(), trunkPosAcc_DSEnd.x());
    traj.get("trunk_pos_y")
        .addPoint(timeDSRightEnd, -trunkPos_DSEnd.y() + footPos_DS.y(), -trunkPosVel_DSEnd.y(), -trunkPosAcc_DSEnd.y());
    traj.get("trunk_pos_z").addPoint(timeDSRightEnd, trunkPos_DSEnd.z(), trunkPosVel_DSEnd.z(), trunkPosAcc_DSEnd.z());
    traj.get("trunk_axis_x")
        .addPoint(timeDSRightEnd, -trunkAxis_DSEnd.x(), -trunkAxisVel_DSEnd.x(), -trunkAxisAcc_DSEnd.x());
    traj.get("trunk_axis_y")
        .addPoint(timeDSRightEnd, trunkAxis_DSEnd.y(), trunkAxisVel_DSEnd.y(), trunkAxisAcc_DSEnd.y());
    traj.get("trunk_axis_z")
        .addPoint(timeDSRightEnd, trunkAxis_DSEnd.z(), trunkAxisVel_DSEnd.z(), trunkAxisAcc_DSEnd.z());
    traj.get("foot_pos_x").addPoint(timeDSRightEnd, -footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSRightEnd, footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSRightEnd, footPos_DS.z(), 0.0, 0.0);

    // Apex at left single support
    traj.get("trunk_pos_x").addPoint(timeSSLeftApex, trunkPos_SS.x(), trunkPosVel_SS.x(), trunkPosAcc_SS.x());
    traj.get("trunk_pos_y").addPoint(timeSSLeftApex, trunkPos_SS.y(), trunkPosVel_SS.y(), trunkPosAcc_SS.y());
    traj.get("trunk_pos_z").addPoint(timeSSLeftApex, trunkPos_SS.z(), trunkPosVel_SS.z(), trunkPosAcc_SS.z());
    traj.get("trunk_axis_x").addPoint(timeSSLeftApex, trunkAxis_SS.x(), trunkAxisVel_SS.x(), trunkAxisAcc_SS.x());
    traj.get("trunk_axis_y").addPoint(timeSSLeftApex, trunkAxis_SS.y(), trunkAxisVel_SS.y(), trunkAxisAcc_SS.y());
    traj.get("trunk_axis_z").addPoint(timeSSLeftApex, trunkAxis_SS.z(), trunkAxisVel_SS.z(), trunkAxisAcc_SS.z());
    traj.get("foot_pos_x").addPoint(timeSSLeftApex, footPos_SS.x(), footPosVel_SS.x(), footPosAcc_SS.x());
    traj.get("foot_pos_y").addPoint(timeSSLeftApex, footPos_SS.y(), footPosVel_SS.y(), footPosAcc_SS.y());
    traj.get("foot_pos_z").addPoint(timeSSLeftApex, footPos_SS.z(), footPosVel_SS.z(), footPosAcc_SS.z());

    // Begin double support left 2
    /*
    traj.get("trunk_pos_x").addPoint(timeDSLeftBegin2, trunkPos_DSBegin.x(), trunkPosVel_DSBegin.x(),
    trunkPosAcc_DSBegin.x()); traj.get("trunk_pos_y").addPoint(timeDSLeftBegin2, trunkPos_DSBegin.y(),
    trunkPosVel_DSBegin.y(), trunkPosAcc_DSBegin.y()); traj.get("trunk_pos_z").addPoint(timeDSLeftBegin2,
    trunkPos_DSBegin.z(), trunkPosVel_DSBegin.z(), trunkPosAcc_DSBegin.z());
    traj.get("trunk_axis_x").addPoint(timeDSLeftBegin2, trunkAxis_DSBegin.x(), trunkAxisVel_DSBegin.x(),
    trunkAxisAcc_DSBegin.x()); traj.get("trunk_axis_y").addPoint(timeDSLeftBegin2, trunkAxis_DSBegin.y(),
    trunkAxisVel_DSBegin.y(), trunkAxisAcc_DSBegin.y()); traj.get("trunk_axis_z").addPoint(timeDSLeftBegin2,
    trunkAxis_DSBegin.z(), trunkAxisVel_DSBegin.z(), trunkAxisAcc_DSBegin.z());
    traj.get("foot_pos_x").addPoint(timeDSLeftBegin2, footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSLeftBegin2, footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSLeftBegin2, footPos_DS.z(), 0.0, 0.0);
    */
    traj.get("trunk_pos_x")
        .addPoint(timeDSLeftBegin2, trunkPos_DSEnd.x(), trunkPosVel_DSEnd.x(), trunkPosAcc_DSEnd.x());
    traj.get("trunk_pos_y")
        .addPoint(timeDSLeftBegin2, trunkPos_DSEnd.y(), trunkPosVel_DSEnd.y(), trunkPosAcc_DSEnd.y());
    traj.get("trunk_pos_z")
        .addPoint(timeDSLeftBegin2, trunkPos_DSEnd.z(), trunkPosVel_DSEnd.z(), trunkPosAcc_DSEnd.z());
    traj.get("trunk_axis_x")
        .addPoint(timeDSLeftBegin2, trunkAxis_DSEnd.x(), trunkAxisVel_DSEnd.x(), trunkAxisAcc_DSEnd.x());
    traj.get("trunk_axis_y")
        .addPoint(timeDSLeftBegin2, trunkAxis_DSEnd.y(), trunkAxisVel_DSEnd.y(), trunkAxisAcc_DSEnd.y());
    traj.get("trunk_axis_z")
        .addPoint(timeDSLeftBegin2, trunkAxis_DSEnd.z(), trunkAxisVel_DSEnd.z(), trunkAxisAcc_DSEnd.z());
    traj.get("foot_pos_x").addPoint(timeDSLeftBegin2, footPos_DS.x(), 0.0, 0.0);
    traj.get("foot_pos_y").addPoint(timeDSLeftBegin2, footPos_DS.y(), 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(timeDSLeftBegin2, footPos_DS.z(), 0.0, 0.0);

    return traj;
  });

  // Set parameters bound function
  generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
    if (params(0) <= 0.1)
    {
      return 1000.0 - 1000.0 * (params(0) - 0.1);
    }
    /*
    if (params(0) >= 0.9) {
        return 1000.0 + 1000.0*(params(0) - 0.9);
    }
    */
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
    (void)dq;
    (void)ddq;

    double cost = 0.0;

    // Compute leg distance from trunk
    double leftDistLeg = std::fabs(model.get().position("left_foot_tip", "trunk").z());
    double rightDistLeg = std::fabs(model.get().position("right_foot_tip", "trunk").z());
    if (leftDistLeg > 0.29)
    {
      cost += 100.0;
    }
    if (leftDistLeg > 0.28)
    {
      return leftDistLeg * 10.0;
    }
    if (rightDistLeg > 0.29)
    {
      cost += 100.0;
    }
    if (rightDistLeg > 0.28)
    {
      return rightDistLeg * 10.0;
    }

    // ZMP
    /*
    if (!isDoubleSupport) {
        Eigen::Vector3d zmp;
        if (supportFoot == Leph::HumanoidFixedModel::LeftSupportFoot) {
            zmp = model.zeroMomentPointSingleSupport("left_foot_tip", torques);
        } else {
            zmp = model.zeroMomentPointSingleSupport("right_foot_tip", torques);
        }
        cost += 10.0*fabs(zmp.x()) + 10.0*fabs(zmp.y());
    }
    */

    // Torques
    Eigen::VectorXd tmpTorques = torques;
    tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
    tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;
    cost += 0.01 * tmpTorques.norm();
    // Maximum torque
    if (data.size() == 0)
    {
      data.push_back(0.0);
      data.push_back(0.0);
    }
    if (data[0] < tmpTorques.lpNorm<Eigen::Infinity>())
    {
      data[0] = tmpTorques.lpNorm<Eigen::Infinity>();
    }

    // Voltage
    for (const std::string& name : Leph::NamesDOF)
    {
      size_t index = model.get().getDOFIndex(name);
      double volt = joints.at(name).computeElectricTension(dq(index), ddq(index), torques(index));
      cost += 0.01 * (1.0 / joints.at(name).getMaxVoltage()) * volt / ((double)Leph::NamesDOF.size());
      // Maximum voltage
      if (data[1] < volt)
      {
        data[1] = volt;
      }
    }

    return cost;
  });
  generator.setEndScoreFunc([](const Eigen::VectorXd& params, const Leph::Trajectories& traj, double score,
                               std::vector<double>& data, bool verbose) -> double {
    (void)params;
    (void)traj;
    (void)score;
    (void)verbose;
    return 5.0 * data[0] + 0.1 * data[1];
  });

  // Display initial trajectory
  TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));
  // Target filename
  std::string filename = "/tmp/trajWalk_" + Leph::currentDate() + ".splines";
  // Run the CMA-ES optimization
  generator.runOptimization(10000, 5, filename, 20, -1.0);
  // Display found trajectory
  TrajectoriesDisplay(generator.bestTrajectories());
}

int main(int argc, char** argv)
{
  // Check command line
  if (argc < 2)
  {
    std::cout << "Usage: ./app [kick|leglift|static|recovery|walk]" << std::endl;
    return 1;
  }
  std::string mode = std::string(argv[1]);

  if (mode == "leglift")
  {
    generateLegLift();
  }
  else if (mode == "kick")
  {
    generateKick();
  }
  else if (mode == "static")
  {
    generateStaticSingleSupport();
  }
  else if (mode == "recovery")
  {
    // generateRecovery();
  }
  else if (mode == "walk")
  {
    // generateWalk();
  }
  else
  {
    std::cout << "Bad mode: " + mode << std::endl;
  }

  return 0;
}
