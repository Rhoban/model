#include <iostream>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidFixedModel.hpp"
#include "Model/JointModel.hpp"
#include "Model/HumanoidSimulation.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"
#include "Model/NamesModel.h"
#include "Utils/FileModelParameters.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"

/**
 * Compare trajectory following between
 * simulated:
 * - simulated no correction
 * - simple feedforward
 * - splines corrections
 */
int main(int argc, char** argv)
{
  // Parsing user inputs
  if (argc != 3 && argc != 5)
  {
    std::cout << "Usage ./app goal.splines corrected.splined"
              << " [MODEL] [path.modelparams]" << std::endl;
    return 1;
  }
  std::string goalFilename = argv[1];
  std::string correctedFilename = argv[2];
  std::string modelFilename = "";
  if (argc == 5 && std::string(argv[3]) == "MODEL")
  {
    modelFilename = argv[4];
  }

  // Load trajectories
  std::cout << "Loading goal trajectory: " << goalFilename << std::endl;
  std::cout << "Loading corrected trajectory: " << correctedFilename << std::endl;
  Leph::Trajectories trajsGoal;
  Leph::Trajectories trajsCorrected;
  trajsGoal.importData(goalFilename);
  trajsCorrected.importData(correctedFilename);
  double timeMin = trajsGoal.min();
  double timeMax = trajsGoal.max();
  std::cout << "TimeMin: " << timeMin << " TimeMax:" << timeMax << std::endl;
  if (fabs(trajsGoal.min() - timeMin) > 1e-9 || fabs(trajsGoal.max() - timeMax) > 1e-9)
  {
    std::cout << "Error incompatible time length" << std::endl;
    return 1;
  }

  // Load model parameters
  Eigen::MatrixXd jointData;
  std::map<std::string, size_t> jointName;
  Eigen::MatrixXd inertiaData;
  std::map<std::string, size_t> inertiaName;
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  if (modelFilename != "")
  {
    std::cout << "Loading model parameters from: " << modelFilename << std::endl;
    Leph::ReadModelParameters(modelFilename, jointData, jointName, inertiaData, inertiaName, geometryData,
                              geometryName);
  }

  // Initialize humanoid model
  // simulations with given
  // model parameters
  Leph::HumanoidSimulation simCorrected(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
  Leph::HumanoidSimulation simFeed(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
  Leph::HumanoidSimulation simGoal(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
  // Assign joint parameters
  for (const std::string& name : Leph::NamesDOF)
  {
    if (jointName.count(name) > 0)
    {
      simCorrected.jointModel(name).setParameters(jointData.row(jointName.at(name)).transpose());
      simFeed.jointModel(name).setParameters(jointData.row(jointName.at(name)).transpose());
      simGoal.jointModel(name).setParameters(jointData.row(jointName.at(name)).transpose());
    }
  }

  // Initialize Humanoid models
  Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
  Leph::HumanoidFixedModel modelCorrected(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);

  // Initialize the viewer
  Leph::ModelViewer viewer(1200, 900);
  Leph::Plot plot;

  // Simulation loop
  bool isInit = true;
  double errorCount = 0.0;
  double errorCorrectedMax = -1.0;
  double errorCorrectedSum = 0.0;
  double errorFeedMax = -1.0;
  double errorFeedSum = 0.0;
  double errorGoalMax = -1.0;
  double errorGoalSum = 0.0;
  for (double t = timeMin; t <= timeMax; t += 0.01)
  {
    if (!viewer.update())
    {
      break;
    }
    // Compute dof target for goal model
    // and goal kinematics
    Eigen::VectorXd dqGoal;
    Eigen::VectorXd ddqGoal;
    bool isSuccessGoal = Leph::TrajectoriesComputeKinematics(t, trajsGoal, modelGoal, dqGoal, ddqGoal);
    if (!isSuccessGoal)
    {
      std::cout << "TrajsGoal IK Error t=" << t << std::endl;
      return 1;
    }
    // Compute external torque on goal trajectories
    // TODO XXX Only handle single support
    Eigen::VectorXd torquesGoal = modelGoal.get().inverseDynamics(dqGoal, ddqGoal);
    // Compute dof target for corrected model
    Eigen::Vector3d trunkPosCorrected;
    Eigen::Vector3d trunkAxisCorrected;
    Eigen::Vector3d footPosCorrected;
    Eigen::Vector3d footAxisCorrected;
    bool isDoubleSupportCorrected;
    Leph::HumanoidFixedModel::SupportFoot supportFootCorrected;
    Leph::TrajectoriesTrunkFootPos(t, trajsCorrected, trunkPosCorrected, trunkAxisCorrected, footPosCorrected,
                                   footAxisCorrected);
    Leph::TrajectoriesSupportFootState(t, trajsCorrected, isDoubleSupportCorrected, supportFootCorrected);
    bool isSuccessCorrected =
        modelCorrected.trunkFootIK(supportFootCorrected, trunkPosCorrected, Leph::AxisToMatrix(trunkAxisCorrected),
                                   footPosCorrected, Leph::AxisToMatrix(footAxisCorrected));
    if (!isSuccessCorrected)
    {
      std::cout << "TrajsCorrected IK Error t=" << t << std::endl;
      return 1;
    }
    // Initialization
    if (isInit)
    {
      isInit = false;
      // Compute expected initial velocities
      // and accelerations
      for (const std::string& name : Leph::NamesDOF)
      {
        simCorrected.setPos(name, modelGoal.get().getDOF(name));
        simCorrected.setGoal(name, modelCorrected.get().getDOF(name));
        simFeed.setPos(name, modelGoal.get().getDOF(name));
        // Compute feed forward
        size_t indexDOF = modelGoal.get().getDOFIndex(name);
        double offset =
            simFeed.jointModel(name).computeFeedForward(dqGoal(indexDOF), ddqGoal(indexDOF), torquesGoal(indexDOF));
        simFeed.setGoal(name, modelGoal.get().getDOF(name) + offset);
        simGoal.setPos(name, modelGoal.get().getDOF(name));
        simGoal.setGoal(name, modelGoal.get().getDOF(name));
        // TODO XXX no handing non zero initial velocity
        simCorrected.setVel(name, 0.0);
        simFeed.setVel(name, 0.0);
        simGoal.setVel(name, 0.0);
        // Reset backlash and goal state
        simCorrected.jointModel(name).resetHiddenState();
        simFeed.jointModel(name).resetHiddenState();
        simGoal.jointModel(name).resetHiddenState();
      }
      // Put the model flat on left support
      // foot at origin
      simCorrected.putOnGround(Leph::HumanoidFixedModel::LeftSupportFoot);
      simFeed.putOnGround(Leph::HumanoidFixedModel::LeftSupportFoot);
      simGoal.putOnGround(Leph::HumanoidFixedModel::LeftSupportFoot);
      simCorrected.putFootAt(0.0, 0.0, Leph::HumanoidFixedModel::LeftSupportFoot);
      simFeed.putFootAt(0.0, 0.0, Leph::HumanoidFixedModel::LeftSupportFoot);
      simGoal.putFootAt(0.0, 0.0, Leph::HumanoidFixedModel::LeftSupportFoot);
      // Run small time 0.5s for
      // waiting stabilization (backlash)
      for (int k = 0; k < 500; k++)
      {
        simCorrected.update(0.001);
        simFeed.update(0.001);
        simGoal.update(0.001);
      }
    }
    // Assign goal to simulation
    for (const std::string& name : Leph::NamesDOF)
    {
      // Corrected spline
      simCorrected.setGoal(name, modelCorrected.get().getDOF(name));
      // Compute feed forward
      size_t indexDOF = modelGoal.get().getDOFIndex(name);
      double offset =
          simFeed.jointModel(name).computeFeedForward(dqGoal(indexDOF), ddqGoal(indexDOF), torquesGoal(indexDOF));
      simFeed.setGoal(name, modelGoal.get().getDOF(name) + offset);
      simGoal.setGoal(name, modelGoal.get().getDOF(name));
    }
    // Simulation step
    for (size_t k = 0; k < 10; k++)
    {
      simCorrected.update(0.001);
      simFeed.update(0.001);
      simGoal.update(0.001);
    }
    // Compute DOF error
    for (const std::string& name : Leph::NamesDOF)
    {
      double errorCorrected =
          180.0 / M_PI * fabs(Leph::AngleDistance(simCorrected.getPos(name), modelGoal.get().getDOF(name)));
      double errorFeed = 180.0 / M_PI * fabs(Leph::AngleDistance(simFeed.getPos(name), modelGoal.get().getDOF(name)));
      double errorGoal = 180.0 / M_PI * fabs(Leph::AngleDistance(simGoal.getPos(name), modelGoal.get().getDOF(name)));
      errorCorrectedSum += errorCorrected;
      errorFeedSum += errorFeed;
      errorGoalSum += errorGoal;
      errorCount += 1.0;
      if (errorCorrectedMax < 0.0 || errorCorrectedMax < errorCorrected)
      {
        errorCorrectedMax = errorCorrected;
      }
      if (errorFeedMax < 0.0 || errorFeedMax < errorFeed)
      {
        errorFeedMax = errorFeed;
      }
      if (errorGoalMax < 0.0 || errorGoalMax < errorGoal)
      {
        errorGoalMax = errorGoal;
      }
      plot.add({
          "t",
          t,
          "target:" + name,
          modelGoal.get().getDOF(name),
          "goal:" + name,
          simGoal.getPos(name),
          "feed:" + name,
          simFeed.getPos(name),
          "corrected:" + name,
          simCorrected.getPos(name),
      });
    }
    // Display
    Leph::ModelDraw(modelGoal.get(), viewer, 1.0);
    Leph::ModelDraw(simCorrected.model(), viewer, 0.6);
    Leph::ModelDraw(simFeed.model(), viewer, 0.4);
    Leph::ModelDraw(simGoal.model(), viewer, 0.2);
  }
  // Show errors
  std::cout << "Corrected   : meanError=" << errorCorrectedSum / errorCount << " maxError=" << errorCorrectedMax
            << std::endl;
  std::cout << "FeedForward : meanError=" << errorFeedSum / errorCount << " maxError=" << errorFeedMax << std::endl;
  std::cout << "OriginalGoal: meanError=" << errorGoalSum / errorCount << " maxError=" << errorGoalMax << std::endl;
  // Plot for each leg DOF
  for (const std::string& name : Leph::NamesDOFLeg)
  {
    plot.plot("t", "target:" + name)
        .plot("t", "goal:" + name)
        .plot("t", "feed:" + name)
        .plot("t", "corrected:" + name)
        .render();
  }

  return 0;
}
