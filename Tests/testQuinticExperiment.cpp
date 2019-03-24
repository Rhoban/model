#include <iostream>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidFixedModel.hpp"
#include "QuinticWalk/QuinticExperiment.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

double scoreWalkDistance(const Leph::QuinticExperiment::Parameters& params, bool display)
{
  double cost = 0.0;

  // Model initialization
  Leph::QuinticExperiment quinticExperiment;
  quinticExperiment.setParameters(params);
  if (display)
  {
    quinticExperiment.getTrajectories().exportData("/tmp/QuinticExperiment.splines");
  }
  Leph::HumanoidFixedModel quinticModel(Leph::SigmabanModel);
  Leph::HumanoidFixedModel ikModel(Leph::SigmabanModel);

  // IK Walk parameter RoboCup config
  Leph::IKWalk::Parameters ikParams;
  ikParams.freq = 1.7;
  ikParams.enabledGain = 1.0;
  ikParams.supportPhaseRatio = 0.0;
  ikParams.footYOffset = 0.025;
  ikParams.stepGain = 0.04;
  ikParams.riseGain = 0.035;
  ikParams.turnGain = 0.0;
  ikParams.lateralGain = 0.0;
  ikParams.swingGain = 0.02;
  ikParams.swingRollGain = 0.0;
  ikParams.swingPhase = 0.25;
  ikParams.stepUpVel = 4.0;
  ikParams.stepDownVel = 4.0;
  ikParams.riseUpVel = 4.0;
  ikParams.riseDownVel = 4.0;
  ikParams.swingPause = 0.0;
  ikParams.swingVel = 4.0;
  ikParams.trunkPitch = 0.15;
  ikParams.trunkRoll = 0.0;
  ikParams.extraLeftX = 0.0;
  ikParams.extraLeftY = 0.0;
  ikParams.extraLeftZ = 0.0;
  ikParams.extraLeftYaw = 0.0;
  ikParams.extraLeftPitch = 0.0;
  ikParams.extraLeftRoll = 0.0;
  ikParams.extraRightX = 0.0;
  ikParams.extraRightY = 0.0;
  ikParams.extraRightZ = 0.0;
  ikParams.extraRightYaw = 0.0;
  ikParams.extraRightPitch = 0.0;
  ikParams.extraRightRoll = 0.0;
  ikParams.trunkXOffset = 0.02;
  ikParams.trunkYOffset = 0.0;
  ikParams.trunkZOffset = 0.02;

  Leph::ModelViewer* viewer = nullptr;
  if (display)
  {
    viewer = new Leph::ModelViewer(1200, 900);
    viewer->maxTrajectory = 1000;
  }
  Leph::Scheduling scheduling(50.0);

  double quinticPhase = 0.5;
  double ikPhase = 0.0;
  bool firstIteration = true;
  double step = 0.01;
  Leph::Plot plot;
  for (double t = 0.0; t < 1.0 / 1.7 || display; t += step)
  {
    // Compute Quintic Experiment
    bool isQuinticSuccess = quinticExperiment.computePose(quinticModel, quinticPhase);
    quinticPhase = quinticExperiment.updatePhase(quinticPhase, step);
    // Compute IK Walk
    bool isIKSuccess = Leph::IKWalk::walk(ikModel.get(), ikParams, ikPhase, step);
    ikModel.updateBase();
    // Reset start position
    if (firstIteration)
    {
      quinticModel.get().setDOF("base_x", 0.0);
      quinticModel.get().setDOF("base_y", 0.0);
      quinticModel.get().setDOF("base_yaw", 0.0);
      ikModel.get().setDOF("base_x", 0.0);
      ikModel.get().setDOF("base_y", 0.0);
      ikModel.get().setDOF("base_yaw", 0.0);
      firstIteration = false;
    }
    // Check IK
    if (!isQuinticSuccess)
    {
      if (display)
      {
        std::cout << "IK ERROR QUINTIC WALK" << std::endl;
      }
      return 1000.0;
    }
    if (!isIKSuccess)
    {
      if (display)
      {
        std::cout << "IK ERROR IK WALK" << std::endl;
      }
      return 1000.0;
    }
    // Compute score
    Eigen::Vector3d ikTrunkPos = ikModel.get().position("trunk", "origin");
    Eigen::Vector3d quinticTrunkPos = quinticModel.get().position("trunk", "origin");
    Eigen::Vector3d ikHeadPos = ikModel.get().position("camera", "origin");
    Eigen::Vector3d quinticHeadPos = quinticModel.get().position("camera", "origin");
    Eigen::Vector3d ikFootRightPos = ikModel.get().position("right_foot_tip", "origin");
    Eigen::Vector3d quinticFootRightPos = quinticModel.get().position("right_foot_tip", "origin");
    Eigen::Vector3d ikFootLeftPos = ikModel.get().position("left_foot_tip", "origin");
    Eigen::Vector3d quinticFootLeftPos = quinticModel.get().position("left_foot_tip", "origin");
    Eigen::Vector3d ikTrunkAxis = Leph::MatrixToAxis(ikModel.get().orientation("trunk", "origin").transpose());
    Eigen::Vector3d quinticTrunkAxis =
        Leph::MatrixToAxis(quinticModel.get().orientation("trunk", "origin").transpose());
    cost += (ikTrunkPos - quinticTrunkPos).squaredNorm();
    cost += (ikHeadPos - quinticHeadPos).squaredNorm();
    cost += (ikFootRightPos - quinticFootRightPos).squaredNorm();
    cost += (ikFootLeftPos - quinticFootLeftPos).squaredNorm();
    cost += (ikTrunkAxis - quinticTrunkAxis).squaredNorm();
    // Display model
    if (display)
    {
      plot.add(Leph::VectorLabel(
          "time", t, "ikFootLeftPos.x", ikFootLeftPos.x(), "ikFootLeftPos.y", ikFootLeftPos.y(), "ikFootLeftPos.z",
          ikFootLeftPos.z(), "ikFootRightPos.x", ikFootRightPos.x(), "ikFootRightPos.y", ikFootRightPos.y(),
          "ikFootRightPos.z", ikFootRightPos.z(), "ikTrunkPos.x", ikTrunkPos.x(), "ikTrunkPos.y", ikTrunkPos.y(),
          "ikTrunkPos.z", ikTrunkPos.z(), "quinticFootLeftPos.x", quinticFootLeftPos.x(), "quinticFootLeftPos.y",
          quinticFootLeftPos.y(), "quinticFootLeftPos.z", quinticFootLeftPos.z(), "quinticFootRightPos.x",
          quinticFootRightPos.x(), "quinticFootRightPos.y", quinticFootRightPos.y(), "quinticFootRightPos.z",
          quinticFootRightPos.z(), "quinticTrunkPos.x", quinticTrunkPos.x(), "quinticTrunkPos.y", quinticTrunkPos.y(),
          "quinticTrunkPos.z", quinticTrunkPos.z()));
      viewer->addTrackedPoint(ikModel.get().position("trunk", "origin"), Leph::ModelViewer::Red);
      viewer->addTrackedPoint(quinticModel.get().position("trunk", "origin"), Leph::ModelViewer::Purple);
      viewer->addTrackedPoint(ikModel.get().position("right_foot_tip", "origin"), Leph::ModelViewer::Blue);
      viewer->addTrackedPoint(quinticModel.get().position("right_foot_tip", "origin"), Leph::ModelViewer::Cyan);
      viewer->addTrackedPoint(ikTrunkPos, Leph::ModelViewer::Green);
      viewer->addTrackedPoint(quinticTrunkPos, Leph::ModelViewer::Yellow);
      Leph::ModelDraw(quinticModel.get(), *viewer);
      Leph::ModelDraw(ikModel.get(), *viewer);
      if (!viewer->update())
      {
        break;
      }
      // Waiting
      scheduling.wait();
    }
  }

  if (display)
  {
    delete viewer;
    plot.plot("time", "all").render();
  }

  return cost;
}

/**
 * Find the QuinticExperiment parameters
 * to match the IKWalk motion
 */
int main()
{
  double lambda = -1.0;
  unsigned int populationSize = 20;
  unsigned int restart = 2;
  unsigned int maxIterations = 1000;

  Leph::QuinticExperiment::Parameters params = Leph::QuinticExperiment::defaultParameters();
  std::cout << scoreWalkDistance(params, true) << std::endl;

  // Fitness function
  libcmaes::FitFuncEigen fitness = [](const Eigen::VectorXd& params) {
    try
    {
      return scoreWalkDistance(params, false);
    }
    catch (...)
    {
      return 1000.0;
    }
  };

  // CMAES initialization
  libcmaes::CMAParameters<> cmaparams(params, lambda, populationSize);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(true);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_elitism(true);
  cmaparams.set_restarts(restart);
  cmaparams.set_max_iter(maxIterations);

  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);

  // Retrieve best Trajectories and score
  params = cmasols.get_best_seen_candidate().get_x_dvec();
  double score = cmasols.get_best_seen_candidate().get_fvalue();

  // Show found parameters
  std::cout << "Best Score: " << score << std::endl;
  std::cout << "Best Params: " << std::endl << params << std::endl;
  scoreWalkDistance(params, true);

  return 0;
}
