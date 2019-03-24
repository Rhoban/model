#include <iostream>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/JointModel.hpp"
#include "TrajectoryGeneration/TrajectoryParameters.hpp"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "TrajectoryDefinition/TrajKickSingle.hpp"
#include "TrajectoryDefinition/TrajKickDouble.hpp"
#include "TrajectoryDefinition/TrajLegLift.hpp"
#include "TrajectoryDefinition/TrajWalk.hpp"
#include "Utils/AxisAngle.h"
#include "Model/NamesModel.h"
#include "Utils/FileModelParameters.h"

/**
 * Optimize a trajectory parameters
 * using the forward simulation as fitness function
 * in order to account for motors control inacuracies.
 */
int main(int argc, char** argv)
{
  // Parse user input
  if (argc < 4)
  {
    std::cout << "Usage: ./app trajectoryName outputPrefix seed.params "
              << "[paramName=value] ... "
              << "[MODEL] [file.modelparams]" << std::endl;
    std::cout << "Available trajectories:" << std::endl;
    std::cout << "-- kicksingle" << std::endl;
    std::cout << "-- kickdouble" << std::endl;
    return 1;
  }
  size_t startInputIndex = 4;
  std::string trajName = argv[1];
  std::string outputPrefix = argv[2];
  std::string seedParametersFile = argv[3];
  // Parse parameters
  std::vector<std::pair<std::string, double>> inputParameters;
  std::string modelParamsPath;
  for (size_t i = startInputIndex; i < (size_t)argc; i++)
  {
    std::string part = argv[i];
    if (part == "MODEL" && i == (size_t)argc - 2)
    {
      modelParamsPath = argv[i + 1];
      std::cout << "Loading model parameters from: " << modelParamsPath << std::endl;
      break;
    }
    size_t pos = part.find("=");
    if (pos == std::string::npos)
    {
      std::cout << "Error format: " << part << std::endl;
      return 1;
    }
    std::string partName = part.substr(0, pos);
    std::string partVal = part.substr(pos + 1);
    if (partName.size() == 0 || partVal.size() == 0)
    {
      std::cout << "Error format: " << part << std::endl;
      return 1;
    }
    inputParameters.push_back({ partName, std::stod(partVal) });
  }

  // Initialize trajectory parameters
  Leph::TrajectoryParameters trajParams = Leph::DefaultTrajParameters();

  // Initialize the generator
  Leph::TrajectoryGeneration generator(Leph::SigmabanModel, modelParamsPath);
  // Load trajectory template and
  // parameter trajectories initialization.
  // Enable forward (complete) parameter optimization.
  if (trajName == "kicksingle")
  {
    Leph::TrajKickSingle::initializeParameters(trajParams, true);
    generator.setTrajectoryGenerationFunc(Leph::TrajKickSingle::funcGeneration(trajParams));
    generator.setCheckParametersFunc(Leph::TrajKickSingle::funcCheckParams(trajParams));
    generator.setCheckDOFFunc(Leph::TrajKickSingle::funcCheckDOF(trajParams));
    generator.setScoreSimFunc(Leph::TrajKickSingle::funcScoreSim(trajParams));
    generator.setEndScoreSimFunc(Leph::TrajKickSingle::funcEndScoreSim(trajParams));
    generator.setSaveFunc(Leph::TrajKickSingle::funcSave(trajParams));
  }
  else if (trajName == "kickdouble")
  {
    Leph::TrajKickDouble::initializeParameters(trajParams, true);
    generator.setTrajectoryGenerationFunc(Leph::TrajKickDouble::funcGeneration(trajParams));
    generator.setCheckParametersFunc(Leph::TrajKickDouble::funcCheckParams(trajParams));
    generator.setCheckDOFFunc(Leph::TrajKickDouble::funcCheckDOF(trajParams));
    generator.setScoreSimFunc(Leph::TrajKickDouble::funcScoreSim(trajParams));
    generator.setEndScoreSimFunc(Leph::TrajKickDouble::funcEndScoreSim(trajParams));
    generator.setSaveFunc(Leph::TrajKickDouble::funcSave(trajParams));
  }
  else
  {
    std::cout << "Invalid trajectory name: " << trajName << std::endl;
    return 1;
  }

  // Load trajectories parameters from file
  std::cout << "Trajectories parameters loading from: " << seedParametersFile << std::endl;
  trajParams.importData(seedParametersFile);

  // Insert inputs parameters to trajectory parameter
  std::string paramsStr = "_";
  for (const auto& it : inputParameters)
  {
    std::cout << "Custom Parameter: " << it.first << "=" << it.second << std::endl;
    trajParams.set(it.first) = it.second;
    // Skip CMAES parameters
    if (it.first.find("cmaes") == std::string::npos)
    {
      paramsStr += it.first + std::string("_") + std::to_string(it.second) + std::string("_");
    }
  }

  // Build initial parameters
  Eigen::VectorXd initParams = trajParams.buildVector();
  // Build normalization coefficents
  Eigen::VectorXd normCoefs = trajParams.buildNormalizationCoefs();

  // Verbose
  char hostnameStr[100];
  gethostname(hostnameStr, 100);
  std::string hostname = hostnameStr;
  std::string filename = outputPrefix + trajName + paramsStr + hostname + "_" + Leph::currentDate();
  std::cout << "Hostname: " << hostname << std::endl;
  std::cout << "Output path: " << filename << std::endl;
  std::cout << "Starting with dimension=" << initParams.size() << std::endl;
  std::cout << "CMA-ES"
            << " max_iterations=" << (unsigned int)trajParams.get("cmaes_max_iterations")
            << " restarts=" << (unsigned int)trajParams.get("cmaes_restarts")
            << " lambda=" << (unsigned int)trajParams.get("cmaes_lambda") << " sigma=" << trajParams.get("cmaes_sigma")
            << " elitism=" << trajParams.get("cmaes_elitism") << std::endl;

  // Set initial parameters
  generator.setInitialParameters(initParams);
  // Set normalization coefficients
  generator.setNormalizationCoefs(normCoefs);

  // Run the CMA-ES optimization
  generator.runOptimization((unsigned int)trajParams.get("cmaes_max_iterations"),
                            (unsigned int)trajParams.get("cmaes_restarts"), filename,
                            (unsigned int)trajParams.get("cmaes_lambda"), trajParams.get("cmaes_sigma"),
                            (unsigned int)trajParams.get("cmaes_elitism"), 10, true);

  return 0;
}
