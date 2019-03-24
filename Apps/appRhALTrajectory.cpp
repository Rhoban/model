#include <iostream>
#include <thread>
#include <chrono>
#include <RhAL.hpp>
#include <RhIO.hpp>
#include "Spline/SplineContainer.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/AxisAngle.h"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/RandomVelocitySpline.hpp"
#include "RhAL/RhALUtils.h"
#include "Types/MapSeries.hpp"

int main(int argc, char** argv)
{
  // Parse command line
  if (argc != 3)
  {
    std::cout << "Usage: ./app [rhal_config_file] [trajectories_file]" << std::endl;
    return 0;
  }
  std::string filenameRhAL = argv[1];
  std::string filenameTrajectories = argv[2];

  // Initialize Trajectories
  Leph::Trajectories trajs;
  std::cout << "Reading Trajectories file: " << filenameTrajectories << std::endl;
  trajs.importData(filenameTrajectories);

  // Initialize the Manager
  RhAL::StandardManager manager;
  std::cout << "Reading RhAL config file: " << filenameRhAL << std::endl;
  std::cout << "RhAL Manager initialization" << std::endl;
  manager.readConfig(filenameRhAL);
  std::cout << "Scanning the bus..." << std::endl;
  manager.scan();

  // Start the Manager
  std::cout << "Starting Manager thread" << std::endl;
  manager.startManagerThread();
  std::cout << "Starting RhIO binding" << std::endl;
  RhAL::RhIOBinding binding(manager);

  // Set up RhIO
  RhIO::Root.newChild("experiment");
  RhIO::Root.newFloat("experiment/trunkPosXOffset")->defaultValue(0.0)->minimum(-0.1)->maximum(0.1);
  RhIO::Root.newFloat("experiment/trunkPosYOffset")->defaultValue(0.0)->minimum(-0.1)->maximum(0.1);
  RhIO::Root.newFloat("experiment/trunkAxisXOffset")->defaultValue(0.0)->minimum(-1.0)->maximum(1.0);
  RhIO::Root.newFloat("experiment/trunkAxisYOffset")->defaultValue(0.0)->minimum(-1.0)->maximum(1.0);
  RhIO::Root.newFloat("experiment/t")->defaultValue(0.0);
  RhIO::Root.newBool("experiment/exploration")->defaultValue(false);
  RhIO::Root.newStr("experiment/state")->defaultValue("idle");
  RhIO::Root.newBool("experiment/continuous")->defaultValue(false);

  // Initialize the model
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);

  // Initialize exploration policy
  /*
  Eigen::VectorXd state(2);
  Eigen::VectorXd minBound(2);
  Eigen::VectorXd maxBound(2);
  state << 0.0, 0.0;
  minBound << -0.03, -0.1;
  maxBound << 0.03, 0.1;
  Leph::RandomVelocitySpline exploration(state, minBound, maxBound, 1.0, 4.0);
  */

  // Start cooperative thread
  std::cout << "Starting User thread" << std::endl;
  manager.enableCooperativeThread();

  RhAL::TimePoint lastTime = RhAL::getTimePoint();
  Leph::MapSeries logs;
  std::cout << "Starting Experiment" << std::endl;
  while (true)
  {
    // Compute elapsed time
    RhAL::TimePoint nowTime = RhAL::getTimePoint();
    double elapsed = RhAL::duration_float(lastTime, nowTime);
    lastTime = nowTime;
    // State
    std::string state = RhIO::Root.getStr("experiment/state");
    if (state == "quit")
    {
      break;
    }
    if (state == "ready")
    {
      RhIO::Root.setStr("experiment/state", "idle");
      double t = trajs.min();
      // Compute Cartesian target
      Eigen::Vector3d trunkPos;
      Eigen::Vector3d trunkAxis;
      Eigen::Vector3d footPos;
      Eigen::Vector3d footAxis;
      bool isDoubleSupport;
      Leph::HumanoidFixedModel::SupportFoot supportFoot;
      Leph::TrajectoriesTrunkFootPos(t, trajs, trunkPos, trunkAxis, footPos, footAxis);
      Leph::TrajectoriesSupportFootState(t, trajs, isDoubleSupport, supportFoot);
      // Apply offsets
      trunkPos.x() += RhIO::Root.getFloat("experiment/trunkPosXOffset");
      trunkPos.y() += RhIO::Root.getFloat("experiment/trunkPosYOffset");
      trunkAxis.x() += RhIO::Root.getFloat("experiment/trunkAxisXOffset");
      trunkAxis.y() += RhIO::Root.getFloat("experiment/trunkAxisYOffset");
      // Compute DOF positions
      bool isSuccess = model.trunkFootIK(supportFoot, trunkPos, Leph::AxisToMatrix(trunkAxis), footPos,
                                         Leph::AxisToMatrix(footAxis));
      // Check IK Success
      if (!isSuccess)
      {
        std::cout << "IK ERROR" << std::endl;
        break;
      }
      // Write target to RhAL with smoothing
      RhALWriteStateGoal(manager, model.get(), true, false, false, true);
    }
    if (state == "play")
    {
      double t = RhIO::Root.getFloat("experiment/t");
      // Check trajectory time bound
      if (t < trajs.min())
      {
        t = trajs.min();
      }
      if (t > trajs.max())
      {
        if (RhIO::Root.getBool("experiment/continuous"))
        {
          while (t > trajs.max())
          {
            t -= trajs.max();
          }
        }
        else
        {
          t = trajs.max();
          RhIO::Root.setStr("experiment/state", "idle");
        }
      }
      // Compute Cartesian target
      Eigen::Vector3d trunkPos;
      Eigen::Vector3d trunkAxis;
      Eigen::Vector3d footPos;
      Eigen::Vector3d footAxis;
      bool isDoubleSupport;
      Leph::HumanoidFixedModel::SupportFoot supportFoot;
      Leph::TrajectoriesTrunkFootPos(t, trajs, trunkPos, trunkAxis, footPos, footAxis);
      Leph::TrajectoriesSupportFootState(t, trajs, isDoubleSupport, supportFoot);
      // Apply offsets
      trunkPos.x() += RhIO::Root.getFloat("experiment/trunkPosXOffset");
      trunkPos.y() += RhIO::Root.getFloat("experiment/trunkPosYOffset");
      trunkAxis.x() += RhIO::Root.getFloat("experiment/trunkAxisXOffset");
      trunkAxis.y() += RhIO::Root.getFloat("experiment/trunkAxisYOffset");
      // Compute DOF positions
      bool isSuccess = model.trunkFootIK(supportFoot, trunkPos, Leph::AxisToMatrix(trunkAxis), footPos,
                                         Leph::AxisToMatrix(footAxis));
      // Check IK Success
      if (!isSuccess)
      {
        std::cout << "IK ERROR" << std::endl;
        break;
      }
      // Update trajectory time
      t += elapsed;
      RhIO::Root.setFloat("experiment/t", t);
      // Write target to RhAL
      RhALWriteStateGoal(manager, model.get(), true, false, false);
    }
    if (state == "idle")
    {
    }

    // Random exploration
    /*
    if (RhIO::Root.getBool("experiment/exploration")) {
        exploration.step(0.02);
        trunkPos.x() += exploration.state()(0);
        trunkAxis.y() += exploration.state()(1);
    }
    */

    // Log goal target and read stats
    Leph::RhALAppendLog(logs, manager);
    // Wait next Manager cycle
    manager.waitNextFlush();
  }
  std::cout << "Stopping Experiment" << std::endl;
  manager.emergencyStop();

  // Save logs
  std::cout << "Writing logs to /tmp/log.series" << std::endl;
  logs.exportData("/tmp/log.series");

  // Stop the manager
  std::cout << "Stopping User thread" << std::endl;
  manager.disableCooperativeThread();
  std::cout << "Stopping Manager thread" << std::endl;
  manager.stopManagerThread();

  return 0;
}
