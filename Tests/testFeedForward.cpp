#include <iostream>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

int main()
{
  // Load trajectories
  std::string filename = "../../These/Data/logs-2016-11-29_model_calibration/"
                         "kick_contact_vel_foot_pos_x_1.000000_cmaes_max_iterations_10000.000000_cmaes_restarts_4."
                         "000000_riker_2016-11-28-21-54-33.splines";
  Leph::Trajectories trajsGoal;
  trajsGoal.importData(filename);

  // Initialize Humanoid models
  Leph::HumanoidModel modelSimNormal(Leph::SigmabanModel, "left_foot_tip", false);
  Leph::HumanoidModel modelSimFeed(Leph::SigmabanModel, "left_foot_tip", false);
  Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);

  // Initialize simulator
  Leph::ForwardSimulation simNormal(modelSimNormal);
  Leph::ForwardSimulation simFeed(modelSimFeed);

  // Initialize the viewer
  Leph::ModelViewer viewer(1200, 900);

  // Time bounds
  double minTime = trajsGoal.min();
  double maxTime = trajsGoal.max();

  // Error varables
  double errorNormalMax = -1.0;
  double errorNormalCount = 0.0;
  double errorNormalSum = 0.0;
  double errorNormalMaxTime = 0.0;
  Eigen::VectorXd errorNormalMaxAll(Leph::NamesDOF.size());
  for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
  {
    errorNormalMaxAll(i) = -1.0;
  }
  std::string errorNormalMaxName = "";
  double errorFeedMax = -1.0;
  double errorFeedCount = 0.0;
  double errorFeedSum = 0.0;
  double errorFeedMaxTime = 0.0;
  Eigen::VectorXd errorFeedMaxAll(Leph::NamesDOF.size());
  for (size_t i = 0; i < Leph::NamesDOF.size(); i++)
  {
    errorFeedMaxAll(i) = -1.0;
  }
  std::string errorFeedMaxName = "";

  // Main loop
  bool isInit = true;
  Leph::Plot plot;
  for (double t = minTime; t <= maxTime; t += 0.01)
  {
    // Viewer update
    if (!viewer.update())
    {
      break;
    }

    // Compute kinematics position, velocity and
    // acceleration on goal trajectories
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
    bool isIKSuccess = Leph::TrajectoriesComputeKinematics(t, trajsGoal, modelGoal, dq, ddq);
    if (!isIKSuccess)
    {
      std::cout << "TrajsGoal IK Error t=" << t << std::endl;
      return 1;
    }
    // Compute external torque on goal trajectories
    Eigen::VectorXd torques = modelGoal.get().inverseDynamics(dq, ddq);

    // Initialization
    if (isInit)
    {
      isInit = false;
      for (const std::string& name : Leph::NamesDOF)
      {
        size_t indexSim = modelSimNormal.getDOFIndex(name);
        simNormal.positions()(indexSim) = modelGoal.get().getDOF(name);
        simNormal.goals()(indexSim) = modelGoal.get().getDOF(name);
        simNormal.velocities()(indexSim) = 0.0;
        simFeed.positions()(indexSim) = modelGoal.get().getDOF(name);
        simFeed.goals()(indexSim) = modelGoal.get().getDOF(name);
        simFeed.velocities()(indexSim) = 0.0;
      }
    }

    // Assign target to simulation
    for (const std::string& name : Leph::NamesDOF)
    {
      size_t indexGoal = modelGoal.get().getDOFIndex(name);
      size_t indexSim = modelSimNormal.getDOFIndex(name);
      // Compute feed forward target angular position offset
      double offset =
          simFeed.jointModel(indexSim).computeFeedForward(dq(indexGoal), ddq(indexGoal), torques(indexGoal));
      simFeed.goals()(indexSim) = modelGoal.get().getDOF(name) + offset;
      // No feed-forward
      simNormal.goals()(indexSim) = modelGoal.get().getDOF(name);
      // Verbose
      plot.add({
          "t",
          t,
          "goal:" + name,
          180.0 / M_PI * modelGoal.get().getDOF(name),
          "feed_goal:" + name,
          180.0 / M_PI * (modelGoal.get().getDOF(name) + offset),
      });
    }

    // Simulation step
    for (size_t k = 0; k < 10; k++)
    {
      simNormal.update(0.001);
      simFeed.update(0.001);
    }

    // Compute error
    size_t index = 0;
    for (const std::string& name : Leph::NamesDOF)
    {
      double errorNormal = pow(180.0 / M_PI * (modelGoal.get().getDOF(name) - modelSimNormal.getDOF(name)), 2);
      errorNormalSum += errorNormal;
      errorNormalCount += 1.0;
      if (errorNormalMax < 0.0 || errorNormalMax < errorNormal)
      {
        errorNormalMax = errorNormal;
        errorNormalMaxTime = t;
        errorNormalMaxName = name;
      }
      if (errorNormalMaxAll(index) < 0.0 || errorNormalMaxAll(index) < errorNormal)
      {
        errorNormalMaxAll(index) = errorNormal;
      }
      double errorFeed = pow(180.0 / M_PI * (modelGoal.get().getDOF(name) - modelSimFeed.getDOF(name)), 2);
      errorFeedSum += errorFeed;
      errorFeedCount += 1.0;
      if (errorFeedMax < 0.0 || errorFeedMax < errorFeed)
      {
        errorFeedMax = errorFeed;
        errorFeedMaxTime = t;
        errorFeedMaxName = name;
      }
      if (errorFeedMaxAll(index) < 0.0 || errorFeedMaxAll(index) < errorFeed)
      {
        errorFeedMaxAll(index) = errorFeed;
      }
      index++;
      plot.add({
          "t",
          t,
          "sim:" + name,
          180.0 / M_PI * modelSimNormal.getDOF(name),
          "feed_sim:" + name,
          180.0 / M_PI * modelSimFeed.getDOF(name),
      });
    }

    // Display
    Leph::ModelDraw(modelSimNormal, viewer, 1.0);
    Leph::ModelDraw(modelSimFeed, viewer, 1.0);
    Leph::ModelDraw(modelGoal.get(), viewer, 0.6);
  }
  std::cout << "========== Without FeedForward:" << std::endl;
  std::cout << "MeanError: " << sqrt(errorNormalSum / errorNormalCount) << std::endl;
  std::cout << "MaxError: " << sqrt(errorNormalMax) << std::endl;
  std::cout << "MaxErrorTime: " << errorNormalMaxTime << std::endl;
  std::cout << "MaxErrorName: " << errorNormalMaxName << std::endl;
  std::cout << "MaxAllErrorMean: " << sqrt(errorNormalMaxAll.mean()) << std::endl;
  std::cout << "MaxAllError: " << (errorNormalMaxAll.array().sqrt().transpose()) << std::endl;
  std::cout << "========== With FeedForward:" << std::endl;
  std::cout << "MeanError: " << sqrt(errorFeedSum / errorFeedCount) << std::endl;
  std::cout << "MaxError: " << sqrt(errorFeedMax) << std::endl;
  std::cout << "MaxErrorTime: " << errorFeedMaxTime << std::endl;
  std::cout << "MaxErrorName: " << errorFeedMaxName << std::endl;
  std::cout << "MaxAllErrorMean: " << sqrt(errorFeedMaxAll.mean()) << std::endl;
  std::cout << "MaxAllError: " << (errorFeedMaxAll.array().sqrt().transpose()) << std::endl;

  for (const std::string& name : Leph::NamesDOF)
  {
    plot.plot("t", "goal:" + name)
        .plot("t", "feed_goal:" + name)
        .plot("t", "sim:" + name)
        .plot("t", "feed_sim:" + name)
        .render();
  }

  return 0;
}
