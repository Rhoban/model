#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "Model/HumanoidSimulation.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/JointModel.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Types/MapSeries.hpp"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"
#include "Utils/FileModelParameters.h"

/**
 * Run HumanoidSimulation
 * on given log or splines
 */
int main(int argc, char** argv)
{
  // Parse arguments
  if (argc < 3 || (argv[1] != std::string("LOG") && argv[1] != std::string("LOG_CYCLE") &&
                   argv[1] != std::string("TRAJ_CYCLE") && argv[1] != std::string("TRAJ")))
  {
    std::cout << "Usage: ./app LOG filepath.mapseries [MODEL] [file.modelparams]" << std::endl;
    std::cout << "Usage: ./app TRAJ filepath.splines [MODEL] [file.modelparams]" << std::endl;
    std::cout << "Usage: ./app LOG_CYCLE filepath.mapseries [MODEL] [file.modelparams]" << std::endl;
    std::cout << "Usage: ./app TRAJ_CYCLE filepath.splines [MODEL] [file.modelparams]" << std::endl;
    return 1;
  }
  std::string filename = argv[2];
  bool isTrajMode;
  bool isCycleMode;
  if (argv[1] == std::string("TRAJ"))
  {
    isTrajMode = true;
    isCycleMode = false;
    std::cout << "Reading episodic Traj: " << filename << std::endl;
  }
  else if (argv[1] == std::string("LOG"))
  {
    isTrajMode = false;
    isCycleMode = false;
    std::cout << "Reading episodic Log: " << filename << std::endl;
  }
  else if (argv[1] == std::string("TRAJ_CYCLE"))
  {
    isTrajMode = true;
    isCycleMode = true;
    std::cout << "Reading periodic Traj: " << filename << std::endl;
  }
  else if (argv[1] == std::string("LOG_CYCLE"))
  {
    isTrajMode = false;
    isCycleMode = true;
    std::cout << "Reading periodic Log: " << filename << std::endl;
  }
  else
  {
    return 0;
  }
  std::string modelParamsPath;
  if (argc == 5 && std::string(argv[3]) == "MODEL")
  {
    modelParamsPath = argv[4];
  }

  // Viewer initialization
  Leph::ModelViewer viewer(1200, 900);
  double timeMin = 0.0;
  double timeMax = 0.0;

  // Load data from trajectories
  Leph::Trajectories trajs;
  if (isTrajMode)
  {
    trajs.importData(filename);
    timeMin = trajs.min();
    timeMax = trajs.max();
    std::cout << "Loading trajs " << filename << ": "
              << "from " << trajs.min() << "s to " << trajs.max() << "s with length " << trajs.max() - trajs.min()
              << "s" << std::endl;
  }

  // Load data into MapSeries
  Leph::MapSeries logs;
  if (!isTrajMode)
  {
    logs.importData(filename);
    timeMin = logs.timeMin();
    timeMax = logs.timeMax();
    std::cout << "Loading log " << filename << ": " << logs.dimension() << " series from " << logs.timeMin() << "s to "
              << logs.timeMax() << "s with length " << logs.timeMax() - logs.timeMin() << "s" << std::endl;
  }

  // Add begin and end waits
  // in non cycle mode
  if (!isCycleMode)
  {
    timeMin -= 0.5;
    timeMax += 2.0;
  }

  // Load model parameters
  Eigen::MatrixXd jointData;
  std::map<std::string, size_t> jointName;
  Eigen::MatrixXd inertiaData;
  std::map<std::string, size_t> inertiaName;
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  if (modelParamsPath != "")
  {
    std::cout << "Loading model parameters from: " << modelParamsPath << std::endl;
    Leph::ReadModelParameters(modelParamsPath, jointData, jointName, inertiaData, inertiaName, geometryData,
                              geometryName);
  }

  // GUI state
  bool isPause = false;
  unsigned int displayMode = 0;
  unsigned int loopIterations = 0;
  unsigned int speedCycle = 1;
  // Main loop
  Leph::Plot plot;
  bool isFirstLoop = true;
  bool isContinue = true;
  while (isContinue)
  {
    // Simulator, goal and read model initialization
    Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
    Leph::HumanoidFixedModel modelRead(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
    Leph::HumanoidSimulation sim(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
    bool isInitialized = false;
    for (const std::string& name : Leph::NamesDOF)
    {
      if (modelParamsPath != "" && jointName.count(name) > 0)
      {
        sim.jointModel(name).setParameters(jointData.row(jointName.at(name)).transpose());
      }
    }
    // Cartesian foot last position
    // for velocity calculations
    Eigen::Vector3d lastFootPosGoal = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastFootPosRead = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastFootPosSim = Eigen::Vector3d::Zero();
    // Simulation loop
    double t = timeMin;
    while (t <= timeMax)
    {
      // Expected velocities and accelerations
      Eigen::VectorXd dq;
      Eigen::VectorXd ddq;
      if (isTrajMode)
      {
        // Compute DOF target and velocities from Cartesian
        bool isSuccess = Leph::TrajectoriesComputeKinematics(t, trajs, modelGoal, dq, ddq);
        if (!isSuccess)
        {
          std::cout << "IK Error t=" << t << std::endl;
          exit(1);
        }
        // Assign simulation DOF goal
        for (const std::string& name : Leph::NamesDOF)
        {
          sim.setGoal(name, modelGoal.get().getDOF(name));
        }
      }
      else
      {
        // Assign simulation DOF goal and read
        for (const std::string& name : Leph::NamesDOF)
        {
          sim.setGoal(name, logs.get("goal:" + name, t));
          modelGoal.get().setDOF(name, logs.get("goal:" + name, t));
          modelRead.get().setDOF(name, logs.get("read:" + name, t));
        }
        // Assign read base state
        modelRead.get().setDOF("base_pitch", logs.get("read:base_pitch", t));
        modelRead.get().setDOF("base_roll", logs.get("read:base_roll", t));
      }
      // State Initialization
      if (!isInitialized)
      {
        // Assign DOF position and velocity
        for (const std::string& name : Leph::NamesDOF)
        {
          sim.setGoal(name, modelGoal.get().getDOF(name));
          sim.setPos(name, modelGoal.get().getDOF(name));
          if (isTrajMode)
          {
            size_t indexModel = modelGoal.get().getDOFIndex(name);
            sim.setVel(name, dq(indexModel));
          }
          else
          {
            sim.setVel(name, 0.0);
          }
          // Reset backlash and goal state
          sim.jointModel(name).resetHiddenState();
        }
        // Put the model flat on left support
        // foot at origin
        sim.putOnGround(Leph::HumanoidFixedModel::LeftSupportFoot);
        sim.putFootAt(0.0, 0.0, Leph::HumanoidFixedModel::LeftSupportFoot);
        if (!isTrajMode)
        {
          // Assign zero base velocity
          for (const std::string& name : Leph::NamesBase)
          {
            sim.setVel(name, 0.0);
          }
        }
        else
        {
          // Compute trunk 6D jacobian with
          // respect to the 6D base DOF
          Eigen::MatrixXd allJac = sim.model().pointJacobian("trunk", "origin");
          Eigen::MatrixXd trunkJac(6, 6);
          trunkJac.col(0) = allJac.col(sim.model().getDOFIndex("base_roll"));
          trunkJac.col(1) = allJac.col(sim.model().getDOFIndex("base_pitch"));
          trunkJac.col(2) = allJac.col(sim.model().getDOFIndex("base_yaw"));
          trunkJac.col(3) = allJac.col(sim.model().getDOFIndex("base_x"));
          trunkJac.col(4) = allJac.col(sim.model().getDOFIndex("base_y"));
          trunkJac.col(5) = allJac.col(sim.model().getDOFIndex("base_z"));
          // Compute 6D target trunk velocities
          Eigen::VectorXd targetTrunkVel = modelGoal.get().pointVelocity("trunk", "origin", dq);
          // Compute base DOF on sim model
          Eigen::VectorXd baseVel = trunkJac.colPivHouseholderQr().solve(targetTrunkVel);
          // Assign base vel
          sim.setVel("base_roll", baseVel(0));
          sim.setVel("base_pitch", baseVel(1));
          sim.setVel("base_yaw", baseVel(2));
          sim.setVel("base_x", baseVel(3));
          sim.setVel("base_y", baseVel(4));
          sim.setVel("base_z", baseVel(5));
        }
        isInitialized = true;
      }
      // Run simulation
      if (!isPause && loopIterations % speedCycle == 0)
      {
        for (int k = 0; k < 10; k++)
        {
          sim.update(0.001);
        }
        sim.printCleatsStatus();
      }
      // Display
      if (displayMode == 0 || displayMode == 2)
      {
        if (!isTrajMode)
        {
          Leph::ModelDraw(modelRead.get(), viewer, 0.3);
        }
        else
        {
          Leph::ModelDraw(modelGoal.get(), viewer, 0.3);
        }
      }
      if (displayMode == 0 || displayMode == 1)
      {
        Leph::ModelDraw(sim.model(), viewer);
        Leph::CleatsDraw(sim, viewer);
      }
      // Viewer update
      if (!viewer.update())
      {
        isContinue = false;
        break;
      }
      // Check user inputs
      if (viewer.isKeyPressed(sf::Keyboard::Space))
      {
        isPause = !isPause;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (viewer.isKeyPressed(sf::Keyboard::M))
      {
        speedCycle++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (viewer.isKeyPressed(sf::Keyboard::P))
      {
        speedCycle--;
        if (speedCycle < 1)
        {
          speedCycle = 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (viewer.isKeyPressed(sf::Keyboard::V))
      {
        displayMode = (displayMode + 1) % 3;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (viewer.isKeyPressed(sf::Keyboard::R))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        isFirstLoop = false;
        break;
      }
      // Plot
      if (isFirstLoop)
      {
        // DOFs data
        for (const std::string& name : Leph::NamesDOF)
        {
          plot.add({
              "t",
              t - timeMin,
              "goal:" + name,
              modelGoal.get().getDOF(name),
              "sim:" + name,
              sim.model().getDOF(name),
          });
          if (!isTrajMode)
          {
            plot.add({
                "t",
                t - timeMin,
                "read:" + name,
                logs.get("read:" + name, t),
            });
          }
        }
        // Cart data
        Eigen::Vector3d trunkPosGoal = modelGoal.get().position("trunk", "left_foot_tip");
        Eigen::Vector3d footPosGoal = modelGoal.get().position("right_foot_tip", "left_foot_tip");
        Eigen::Vector3d trunkPosSim = sim.model().position("trunk", "left_foot_tip");
        Eigen::Vector3d footPosSim = sim.model().position("right_foot_tip", "left_foot_tip");
        // Compute foot velocity by differentiation
        Eigen::Vector3d footVelGoal = (1.0 / 0.01) * (footPosGoal - lastFootPosGoal);
        Eigen::Vector3d footVelSim = (1.0 / 0.01) * (footPosSim - lastFootPosSim);
        lastFootPosGoal = footPosGoal;
        lastFootPosSim = footPosSim;
        plot.add({
            "t",
            t - timeMin,
            "goal:trunk_x",
            trunkPosGoal.x(),
            "goal:trunk_y",
            trunkPosGoal.y(),
            "goal:trunk_z",
            trunkPosGoal.z(),
            "goal:foot_x",
            footPosGoal.x(),
            "goal:foot_y",
            footPosGoal.y(),
            "goal:foot_z",
            footPosGoal.z(),
            "sim:trunk_x",
            trunkPosSim.x(),
            "sim:trunk_y",
            trunkPosSim.y(),
            "sim:trunk_z",
            trunkPosSim.z(),
            "sim:foot_x",
            footPosSim.x(),
            "sim:foot_y",
            footPosSim.y(),
            "sim:foot_z",
            footPosSim.z(),
        });
        plot.add({
            "t",
            t - timeMin,
            "vel_goal:foot_x",
            footVelGoal.x(),
            "vel_goal:foot_y",
            footVelGoal.y(),
            "vel_goal:foot_z",
            footVelGoal.z(),
            "vel_sim:foot_x",
            footVelSim.x(),
            "vel_sim:foot_y",
            footVelSim.y(),
            "vel_sim:foot_z",
            footVelSim.z(),
        });
        if (!isTrajMode)
        {
          Eigen::Vector3d trunkPosRead = modelRead.get().position("trunk", "left_foot_tip");
          Eigen::Vector3d footPosRead = modelRead.get().position("right_foot_tip", "left_foot_tip");
          Eigen::Vector3d footVelRead = (1.0 / 0.01) * (footPosRead - lastFootPosRead);
          lastFootPosRead = footPosRead;
          plot.add({
              "t",
              t - timeMin,
              "read:trunk_x",
              trunkPosRead.x(),
              "read:trunk_y",
              trunkPosRead.y(),
              "read:trunk_z",
              trunkPosRead.z(),
              "read:foot_x",
              footPosRead.x(),
              "read:foot_y",
              footPosRead.y(),
              "read:foot_z",
              footPosRead.z(),
          });
          plot.add({
              "t",
              t - timeMin,
              "vel_read:foot_x",
              footVelRead.x(),
              "vel_read:foot_y",
              footVelRead.y(),
              "vel_read:foot_z",
              footVelRead.z(),
          });
        }
      }
      if (!isPause && loopIterations % speedCycle == 0)
      {
        // Time update
        t += 0.01;
      }
      loopIterations++;
      // Cycle mode
      if (isCycleMode && t > timeMax)
      {
        t -= timeMax - timeMin;
        isFirstLoop = false;
      }
    }
    isFirstLoop = false;
  }
  // Plot
  plot.plot("t", "goal:left_hip_yaw")
      .plot("t", "read:left_hip_yaw")
      .plot("t", "sim:left_hip_yaw")
      .plot("t", "goal:left_hip_pitch")
      .plot("t", "read:left_hip_pitch")
      .plot("t", "sim:left_hip_pitch")
      .plot("t", "goal:left_hip_roll")
      .plot("t", "read:left_hip_roll")
      .plot("t", "sim:left_hip_roll")
      .plot("t", "goal:left_knee")
      .plot("t", "read:left_knee")
      .plot("t", "sim:left_knee")
      .plot("t", "goal:left_ankle_pitch")
      .plot("t", "read:left_ankle_pitch")
      .plot("t", "sim:left_ankle_pitch")
      .plot("t", "goal:left_ankle_roll")
      .plot("t", "read:left_ankle_roll")
      .plot("t", "sim:left_ankle_roll")
      .render();
  plot.plot("t", "goal:right_hip_yaw")
      .plot("t", "read:right_hip_yaw")
      .plot("t", "sim:right_hip_yaw")
      .plot("t", "goal:right_hip_pitch")
      .plot("t", "read:right_hip_pitch")
      .plot("t", "sim:right_hip_pitch")
      .plot("t", "goal:right_hip_roll")
      .plot("t", "read:right_hip_roll")
      .plot("t", "sim:right_hip_roll")
      .plot("t", "goal:right_knee")
      .plot("t", "read:right_knee")
      .plot("t", "sim:right_knee")
      .plot("t", "goal:right_ankle_pitch")
      .plot("t", "read:right_ankle_pitch")
      .plot("t", "sim:right_ankle_pitch")
      .plot("t", "goal:right_ankle_roll")
      .plot("t", "read:right_ankle_roll")
      .plot("t", "sim:right_ankle_roll")
      .render();
  plot.plot("t", "goal:trunk_x")
      .plot("t", "read:trunk_x")
      .plot("t", "sim:trunk_x")
      .plot("t", "goal:trunk_y")
      .plot("t", "read:trunk_y")
      .plot("t", "sim:trunk_y")
      .plot("t", "goal:trunk_z")
      .plot("t", "read:trunk_z")
      .plot("t", "sim:trunk_z")
      .render();
  plot.plot("t", "goal:foot_x")
      .plot("t", "read:foot_x")
      .plot("t", "sim:foot_x")
      .plot("t", "goal:foot_y")
      .plot("t", "read:foot_y")
      .plot("t", "sim:foot_y")
      .plot("t", "goal:foot_z")
      .plot("t", "read:foot_z")
      .plot("t", "sim:foot_z")
      .render();
  plot.plot("t", "goal:foot_x")
      .plot("t", "vel_goal:foot_x")
      .plot("t", "read:foot_x")
      .plot("t", "vel_read:foot_x")
      .plot("t", "sim:foot_x")
      .plot("t", "vel_sim:foot_x")
      .render();

  return 0;
}
