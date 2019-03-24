#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidSimulation.hpp"
#include "Model/JointModel.hpp"
#include "Types/MapSeries.hpp"
#include "Utils/Angle.h"
#include "Model/NamesModel.h"
#include "Utils/FileModelParameters.h"

#ifdef LEPH_VIEWER_ENABLED
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"
#endif

/**
 * Names of DOF whose Joint Model is uniquely optimized
 */
static const std::vector<std::string> namesDOFJoint = {
  "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
  "left_hip_yaw",  "left_hip_pitch",  "left_hip_roll",  "left_knee",  "left_ankle_pitch",  "left_ankle_roll",
};

/**
 * Names of frame whose inertia data is optimized
 */
static const std::vector<std::string> namesFrameInertia = {
  "trunk",           "right_hip_yaw",     "right_hip_pitch",  "right_hip_roll",
  "right_knee",      "right_ankle_pitch", "right_ankle_roll", "left_hip_yaw",
  "left_hip_pitch",  "left_hip_roll",     "left_knee",        "left_ankle_pitch",
  "left_ankle_roll",
};

/**
 * Names of frame whose geometry data is optimized
 */
static const std::vector<std::string> namesFrameGeometry = {
  "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
  "left_hip_yaw",  "left_hip_pitch",  "left_hip_roll",  "left_knee",  "left_ankle_pitch",  "left_ankle_roll",
};

/**
 * Global CMA-ES configuration
 */
static const int cmaesElitismLevel = 0;
static const unsigned int cmaesMaxIterations = 2000;
static const unsigned int cmaesRestarts = 5;
static const unsigned int cmaesLambda = 100;
static const double cmaesSigma = -1.0;

/**
 * Global weighting ratio
 * on trunk orientation errors
 */
static const double trunkOrientationCoef = 0.5;

/**
 * Simulate and compute the error between given logs
 * sequence and simulated model with given parameters.
 */
static double scoreFitness(
    const Leph::MapSeries& logs, const Eigen::VectorXd& parameters, size_t indexStartCommon, size_t indexStartJoints,
    size_t indexStartInertias, size_t indexStartGeometries, size_t sizeJointParameters, size_t sizeInertiaParameters,
    size_t sizeGeometryParameters, const Eigen::MatrixXd& defaultJointData,
    const std::map<std::string, size_t>& defaultJointName, const Eigen::MatrixXd& defaultInertiaData,
    const std::map<std::string, size_t>& defaultInertiaName, const Eigen::MatrixXd& defaultGeometryData,
    const std::map<std::string, size_t>& defaultGeometryName, int verbose, double* sumError = nullptr,
    double* countError = nullptr, double* maxError = nullptr, Eigen::VectorXd* maxAllError = nullptr)
{
  // Retrieve log time bounds
  double minTime = logs.timeMin();
  double maxTime = logs.timeMax();

  // Build used inertia data
  Eigen::MatrixXd currentInertiaData = defaultInertiaData;
  if (indexStartInertias != (size_t)-1)
  {
    size_t index = 0;
    for (const std::string& name : namesFrameInertia)
    {
      currentInertiaData.row(defaultInertiaName.at(name)) =
          parameters.segment(indexStartInertias + index * sizeInertiaParameters, sizeInertiaParameters).transpose();
      index++;
    }
  }
  // Build used geometry data
  Eigen::MatrixXd currentGeometryData = defaultGeometryData;
  if (indexStartGeometries != (size_t)-1)
  {
    size_t index = 0;
    for (const std::string& name : namesFrameGeometry)
    {
      currentGeometryData.row(defaultGeometryName.at(name)) =
          parameters.segment(indexStartGeometries + index * sizeGeometryParameters, sizeGeometryParameters).transpose();
      index++;
    }
  }

  // Initialize full humanoid model
  // simulation with overrided
  // inertia and geometry data
  Leph::HumanoidSimulation sim(Leph::SigmabanModel, currentInertiaData, defaultInertiaName, currentGeometryData,
                               defaultGeometryName);
  Leph::HumanoidModel modelRead(Leph::SigmabanModel, "left_foot_tip", true, currentInertiaData, defaultInertiaName,
                                currentGeometryData, defaultGeometryName);

  // Assign common joint parameters
  if (indexStartCommon != (size_t)-1)
  {
    sim.setJointModelParameters(parameters.segment(indexStartCommon, sizeJointParameters));
  }
  else
  {
    for (const std::string& name : namesDOFJoint)
    {
      sim.jointModel(name).setParameters(defaultJointData.row(defaultJointName.at(name)).transpose());
    }
  }
  // Assign joint parameters to uniquely optimized DOF
  if (indexStartJoints != (size_t)-1)
  {
    size_t index = 0;
    for (const std::string& name : namesDOFJoint)
    {
      sim.jointModel(name).setParameters(
          parameters.segment(indexStartJoints + index * sizeJointParameters, sizeJointParameters));
      index++;
    }
  }

  // State initialization
  for (const std::string& name : Leph::NamesDOF)
  {
    // Initialization pos, vel and goal
    sim.setPos(name, logs.get("read:" + name, minTime));
    sim.setGoal(name, logs.get("read:" + name, minTime));
    sim.setVel(name, 0.0);
    // Reset backlash and goal state
    sim.jointModel(name).resetHiddenState();
  }
  for (const std::string& name : Leph::NamesBase)
  {
    // Init base vel
    sim.setVel(name, 0.0);
  }
  // Init model state
  sim.putOnGround(Leph::HumanoidFixedModel::LeftSupportFoot);
  sim.putFootAt(0.0, 0.0, Leph::HumanoidFixedModel::LeftSupportFoot);
  // Run small time 0.5s for
  // waiting stabilization (backlash)
  for (int k = 0; k < 500; k++)
  {
    sim.update(0.001);
  }

  // Main loop
  double tmpMax = -1.0;
  double tmpCount = 0.0;
  double tmpSum = 0.0;
  double tmpMaxTime = 0.0;
  Eigen::VectorXd tmpMaxAll(Leph::NamesDOF.size() + 2);
  for (size_t i = 0; i < Leph::NamesDOF.size() + 2; i++)
  {
    tmpMaxAll(i) = -1.0;
  }
  std::string tmpMaxName = "";
#ifdef LEPH_VIEWER_ENABLED
  Leph::ModelViewer* viewer = nullptr;
  if (verbose >= 3)
  {
    viewer = new Leph::ModelViewer(1200, 900);
  }
  Leph::Plot plot;
#endif
  double incrStep = 0.01;
  int incrLoop = 10;
  for (double t = minTime; t < maxTime; t += incrStep)
  {
#ifdef LEPH_VIEWER_ENABLED
    if (verbose >= 3)
    {
      if (!viewer->update())
      {
        break;
      }
    }
#endif
    // Assign motor goal
    for (const std::string& name : Leph::NamesDOF)
    {
      sim.setGoal(name, logs.get("goal:" + name, t));
      modelRead.setDOF(name, logs.get("read:" + name, t));
    }
    modelRead.setDOF("base_roll", logs.get("read:base_roll", t));
    modelRead.setDOF("base_pitch", logs.get("read:base_pitch", t));
    // Run simulation
    for (int k = 0; k < incrLoop; k++)
    {
      sim.update(0.001);
    }
    // Compute DOF error
    size_t index = 0;
    for (const std::string& name : Leph::NamesDOF)
    {
      double error = pow(180.0 / M_PI * Leph::AngleDistance(sim.getPos(name), modelRead.getDOF(name)), 2);
      tmpSum += error;
      tmpCount += 1.0;
      if (tmpMax < 0.0 || tmpMax < error)
      {
        tmpMax = error;
        tmpMaxTime = t - minTime;
        tmpMaxName = name;
      }
      if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < error)
      {
        tmpMaxAll(index) = error;
      }
      index++;
    }
    // Compute trunk orientation error
    Eigen::Vector3d simTrunkAngles = sim.model().trunkSelfOrientation();
    double errorTrunkRoll = pow(
        trunkOrientationCoef * 180.0 / M_PI * Leph::AngleDistance(simTrunkAngles.x(), logs.get("read:imu_roll", t)), 2);
    double errorTrunkPitch = pow(trunkOrientationCoef * 180.0 / M_PI *
                                     Leph::AngleDistance(simTrunkAngles.y(), logs.get("read:imu_pitch", t)),
                                 2);
    tmpSum += errorTrunkRoll;
    tmpSum += errorTrunkPitch;
    tmpCount += 2.0;
    if (tmpMax < 0.0 || tmpMax < errorTrunkRoll)
    {
      tmpMax = errorTrunkRoll;
      tmpMaxTime = t - minTime;
      tmpMaxName = "trunk_roll";
    }
    if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < errorTrunkRoll)
    {
      tmpMaxAll(index) = errorTrunkRoll;
    }
    index++;
    if (tmpMax < 0.0 || tmpMax < errorTrunkPitch)
    {
      tmpMax = errorTrunkPitch;
      tmpMaxTime = t - minTime;
      tmpMaxName = "trunk_pitch";
    }
    if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < errorTrunkPitch)
    {
      tmpMaxAll(index) = errorTrunkPitch;
    }
    index++;
    // Verbose Plot
#ifdef LEPH_VIEWER_ENABLED
    Leph::VectorLabel vect;
    if (verbose >= 2)
    {
      for (const std::string& name : Leph::NamesDOF)
      {
        vect.setOrAppend("t", t);
        vect.setOrAppend("read:" + name, 180.0 / M_PI * logs.get("read:" + name, t));
        vect.setOrAppend("goal:" + name, 180.0 / M_PI * logs.get("goal:" + name, t));
        vect.setOrAppend("sim:" + name, 180.0 / M_PI * sim.getPos(name));
      }
      vect.setOrAppend("read:trunk_pitch", 180.0 / M_PI * logs.get("read:imu_pitch", t));
      vect.setOrAppend("read:trunk_roll", 180.0 / M_PI * logs.get("read:imu_roll", t));
      vect.setOrAppend("sim:trunk_roll", 180.0 / M_PI * simTrunkAngles.x());
      vect.setOrAppend("sim:trunk_pitch", 180.0 / M_PI * simTrunkAngles.y());
      plot.add(vect);
    }
    if (verbose >= 3)
    {
      Leph::CleatsDraw(sim, *viewer);
      Leph::ModelDraw(sim.model(), *viewer, 1.0);
      Leph::ModelDraw(modelRead, *viewer, 0.5);
    }
#endif
  }

  if (verbose >= 1)
  {
    std::cout << "MeanError: " << sqrt(tmpSum / tmpCount) << std::endl;
    std::cout << "MaxError: " << sqrt(tmpMax) << std::endl;
    std::cout << "MaxErrorTime: " << tmpMaxTime << std::endl;
    std::cout << "MaxErrorName: " << tmpMaxName << std::endl;
    std::cout << "MaxAllErrorMean: " << sqrt(tmpMaxAll.mean()) << std::endl;
    std::cout << "MaxAllError: " << (tmpMaxAll.array().sqrt().transpose()) << std::endl;
  }
#ifdef LEPH_VIEWER_ENABLED
  if (verbose >= 2)
  {
    plot.plot("t", "read:left_ankle_roll")
        .plot("t", "goal:left_ankle_roll")
        .plot("t", "sim:left_ankle_roll")
        .plot("t", "read:left_ankle_pitch")
        .plot("t", "goal:left_ankle_pitch")
        .plot("t", "sim:left_ankle_pitch")
        .plot("t", "read:left_knee")
        .plot("t", "goal:left_knee")
        .plot("t", "sim:left_knee")
        .plot("t", "read:left_hip_pitch")
        .plot("t", "goal:left_hip_pitch")
        .plot("t", "sim:left_hip_pitch")
        .plot("t", "read:left_hip_roll")
        .plot("t", "goal:left_hip_roll")
        .plot("t", "sim:left_hip_roll")
        .plot("t", "read:left_hip_yaw")
        .plot("t", "goal:left_hip_yaw")
        .plot("t", "sim:left_hip_yaw")
        .render();
    plot.plot("t", "read:right_ankle_roll")
        .plot("t", "goal:right_ankle_roll")
        .plot("t", "sim:right_ankle_roll")
        .plot("t", "read:right_ankle_pitch")
        .plot("t", "goal:right_ankle_pitch")
        .plot("t", "sim:right_ankle_pitch")
        .plot("t", "read:right_knee")
        .plot("t", "goal:right_knee")
        .plot("t", "sim:right_knee")
        .plot("t", "read:right_hip_pitch")
        .plot("t", "goal:right_hip_pitch")
        .plot("t", "sim:right_hip_pitch")
        .plot("t", "read:right_hip_roll")
        .plot("t", "goal:right_hip_roll")
        .plot("t", "sim:right_hip_roll")
        .plot("t", "read:right_hip_yaw")
        .plot("t", "goal:right_hip_yaw")
        .plot("t", "sim:right_hip_yaw")
        .render();
    plot.plot("t", "read:trunk_pitch")
        .plot("t", "read:trunk_roll")
        .plot("t", "sim:trunk_pitch")
        .plot("t", "sim:trunk_roll")
        .render();
  }
  if (verbose >= 3)
  {
    delete viewer;
  }
#endif
  if (sumError != nullptr)
  {
    *sumError += tmpSum;
  }
  if (countError != nullptr)
  {
    *countError += tmpCount;
  }
  if (maxError != nullptr && (*maxError < 0.0 || *maxError < tmpMax))
  {
    *maxError = tmpMax;
  }
  if (maxAllError != nullptr)
  {
    for (size_t i = 0; i < Leph::NamesDOF.size() + 2; i++)
    {
      if (tmpMaxAll(i) > maxAllError->operator()(i))
      {
        maxAllError->operator()(i) = tmpMaxAll(i);
      }
    }
  }

  return 0.0;
}

/**
 * Dump to given filename the model parameters contained
 * in given parameters vector and in default
 * joint, inertia and geometry matrix.
 */
static void saveModelParameters(const std::string& filename, const Eigen::VectorXd& parameters, size_t indexStartCommon,
                                size_t indexStartJoints, size_t indexStartInertias, size_t indexStartGeometries,
                                size_t sizeJointParameters, size_t sizeInertiaParameters, size_t sizeGeometryParameters,
                                const Eigen::MatrixXd& defaultJointData,
                                const std::map<std::string, size_t>& defaultJointName,
                                const Eigen::MatrixXd& defaultInertiaData,
                                const std::map<std::string, size_t>& defaultInertiaName,
                                const Eigen::MatrixXd& defaultGeometryData,
                                const std::map<std::string, size_t>& defaultGeometryName)
{
  Eigen::MatrixXd currentJointData = defaultJointData;
  Eigen::MatrixXd currentInertiaData = defaultInertiaData;
  Eigen::MatrixXd currentGeometryData = defaultGeometryData;

  if (indexStartCommon != (size_t)-1)
  {
    for (const auto& it : defaultJointName)
    {
      currentJointData.row(defaultJointName.at(it.first)) =
          parameters.segment(indexStartCommon, sizeJointParameters).transpose();
    }
  }
  if (indexStartJoints != (size_t)-1)
  {
    size_t index = 0;
    for (const std::string& name : namesDOFJoint)
    {
      currentJointData.row(defaultJointName.at(name)) =
          parameters.segment(indexStartJoints + index * sizeJointParameters, sizeJointParameters).transpose();
      index++;
    }
  }
  if (indexStartInertias != (size_t)-1)
  {
    size_t index = 0;
    for (const std::string& name : namesFrameInertia)
    {
      currentInertiaData.row(defaultInertiaName.at(name)) =
          parameters.segment(indexStartInertias + index * sizeInertiaParameters, sizeInertiaParameters).transpose();
      index++;
    }
  }
  if (indexStartGeometries != (size_t)-1)
  {
    size_t index = 0;
    for (const std::string& name : namesFrameGeometry)
    {
      currentGeometryData.row(defaultGeometryName.at(name)) =
          parameters.segment(indexStartGeometries + index * sizeGeometryParameters, sizeGeometryParameters).transpose();
      index++;
    }
  }

  Leph::WriteModelParameters(filename, currentJointData, defaultJointName, currentInertiaData, defaultInertiaName,
                             currentGeometryData, defaultGeometryName);
}

/**
 * Print application usage
 */
static void printUsage()
{
  std::cout << "Usage: "
            << "./app output.modelparams mode ... "
            << "LEARNING logfile.mapseries ... "
            << "VALIDATION logfile.mapseries ... "
            << "SEED inputSeed.modelparams" << std::endl;
  std::cout << "Available optimization modes:" << std::endl;
  std::cout << "-- COMMON" << std::endl;
  std::cout << "-- JOINTS" << std::endl;
  std::cout << "-- INERTIAS" << std::endl;
  std::cout << "-- GEOMETRIES" << std::endl;
}

/**
 * Use CMA-ES optimization to find
 * joint parameters and/or intertia
 * parameters for which forward simulation
 * to match logged motion
 */
int main(int argc, char** argv)
{
  // Parse user inputs
  if (argc < 5)
  {
    printUsage();
    return 1;
  }
  // Retrieve output model parameter filename
  std::string outputParamsFilename = argv[1];
  // Retrieve optimization option
  int argIndex = 2;
  bool isIdentificationCommon = false;
  bool isIdentificationJoints = false;
  bool isIdentificationInertias = false;
  bool isIdentificationGeometries = false;
  while (argIndex < argc && std::string(argv[argIndex]) != "LEARNING")
  {
    if (std::string(argv[argIndex]) == "COMMON")
    {
      isIdentificationCommon = true;
    }
    else if (std::string(argv[argIndex]) == "JOINTS")
    {
      isIdentificationJoints = true;
    }
    else if (std::string(argv[argIndex]) == "INERTIAS")
    {
      isIdentificationInertias = true;
    }
    else if (std::string(argv[argIndex]) == "GEOMETRIES")
    {
      isIdentificationGeometries = true;
    }
    else
    {
      std::cout << "Invalid optimization option." << std::endl;
      printUsage();
      return 1;
    }
    argIndex++;
  }
  // Retrieve learning log filenames
  argIndex++;
  std::vector<std::string> filenames;
  while (argIndex < argc && std::string(argv[argIndex]) != "VALIDATION")
  {
    filenames.push_back(std::string(argv[argIndex]));
    argIndex++;
  }
  // Retrieve validation log filenames
  argIndex++;
  std::vector<std::string> filenamesValidation;
  while (argIndex < argc && std::string(argv[argIndex]) != "SEED")
  {
    filenamesValidation.push_back(std::string(argv[argIndex]));
    argIndex++;
  }
  // Retrieve seed input model parameters
  argIndex++;
  std::string inputParamsFilename;
  if (argIndex < argc)
  {
    inputParamsFilename = argv[argIndex];
  }

  // Load learning data logs into MapSeries
  std::vector<Leph::MapSeries> logsData;
  for (size_t i = 0; i < filenames.size(); i++)
  {
    logsData.push_back(Leph::MapSeries());
    logsData.back().importData(filenames[i]);
    std::cout << "Loaded learning " << filenames[i] << ": " << logsData.back().dimension() << " series from "
              << logsData.back().timeMin() << "s to " << logsData.back().timeMax() << "s with length "
              << logsData.back().timeMax() - logsData.back().timeMin() << "s" << std::endl;
  }
  // Load validation data logs into MapSeries
  std::vector<Leph::MapSeries> logsValidation;
  for (size_t i = 0; i < filenamesValidation.size(); i++)
  {
    logsValidation.push_back(Leph::MapSeries());
    logsValidation.back().importData(filenamesValidation[i]);
    std::cout << "Loaded validation " << filenamesValidation[i] << ": " << logsValidation.back().dimension()
              << " series from " << logsValidation.back().timeMin() << "s to " << logsValidation.back().timeMax()
              << "s with length " << logsValidation.back().timeMax() - logsValidation.back().timeMin() << "s"
              << std::endl;
  }

  // Inertia default data and name
  Eigen::MatrixXd defaultInertiaData;
  std::map<std::string, size_t> defaultInertiaName;
  // Geometry default data and name
  Eigen::MatrixXd defaultGeometryData;
  std::map<std::string, size_t> defaultGeometryName;
  // Load default inertia and geometry data from model
  Leph::HumanoidModel tmpModel(Leph::SigmabanModel, "left_foot_tip", false);
  defaultInertiaData = tmpModel.getInertiaData();
  defaultInertiaName = tmpModel.getInertiaName();
  defaultGeometryData = tmpModel.getGeometryData();
  defaultGeometryName = tmpModel.getGeometryName();

  // Joint default parameters
  Eigen::VectorXd defaultJointParams;
  // Load default joint model parameters
  Leph::JointModel tmpJoint;
  defaultJointParams = tmpJoint.getParameters();

  // Build default joint parameters matrix
  Eigen::MatrixXd defaultJointData(Leph::NamesDOF.size(), defaultJointParams.size());
  std::map<std::string, size_t> defaultJointName;
  size_t tmpIndex = 0;
  for (const std::string& name : Leph::NamesDOF)
  {
    defaultJointName[name] = tmpIndex;
    defaultJointData.block(tmpIndex, 0, 1, defaultJointParams.size()) = defaultJointParams.transpose();
    tmpIndex++;
  }

  // Load if provided the seed model parameters
  if (inputParamsFilename != "")
  {
    std::cout << "Loading seed model parameters from: " << inputParamsFilename << std::endl;
    Leph::ReadModelParameters(inputParamsFilename, defaultJointData, defaultJointName, defaultInertiaData,
                              defaultInertiaName, defaultGeometryData, defaultGeometryName);
  }

  // Build initial parameters vector
  Eigen::VectorXd initParams = defaultJointParams;
  size_t indexStartCommon = (size_t)-1;
  size_t indexStartJoints = (size_t)-1;
  size_t indexStartInertias = (size_t)-1;
  size_t indexStartGeometries = (size_t)-1;
  // Util sizes
  size_t sizeJointParameters = defaultJointParams.size();
  size_t sizeInertiaParameters = defaultInertiaData.cols();
  size_t sizeGeometryParameters = defaultGeometryData.cols();
  size_t sizeAllParameters = 0;
  // Add common optimized joint parameters
  if (isIdentificationCommon)
  {
    indexStartCommon = sizeAllParameters;
    sizeAllParameters += sizeJointParameters;
    initParams.conservativeResize(sizeAllParameters);
    initParams.segment(indexStartCommon, sizeJointParameters) = defaultJointParams;
  }
  // Add all uniquely optimized joint parameters
  if (isIdentificationJoints)
  {
    indexStartJoints = sizeAllParameters;
    sizeAllParameters += namesDOFJoint.size() * sizeJointParameters;
    initParams.conservativeResize(sizeAllParameters);
    size_t index = 0;
    for (const std::string& name : namesDOFJoint)
    {
      initParams.segment(indexStartJoints + index * sizeJointParameters, sizeJointParameters) =
          defaultJointData.row(defaultJointName.at(name)).transpose();
      index++;
    }
  }
  // Add all uniquely optimized inertia parameters
  if (isIdentificationInertias)
  {
    indexStartInertias = sizeAllParameters;
    sizeAllParameters += namesFrameInertia.size() * sizeInertiaParameters;
    initParams.conservativeResize(sizeAllParameters);
    size_t index = 0;
    for (const std::string& name : namesFrameInertia)
    {
      initParams.segment(indexStartInertias + index * sizeInertiaParameters, sizeInertiaParameters) =
          defaultInertiaData.row(defaultInertiaName.at(name)).transpose();
      index++;
    }
  }
  // Add all uniquely optimized geometry parameters
  if (isIdentificationGeometries)
  {
    indexStartGeometries = sizeAllParameters;
    sizeAllParameters += namesFrameGeometry.size() * sizeGeometryParameters;
    initParams.conservativeResize(sizeAllParameters);
    size_t index = 0;
    for (const std::string& name : namesFrameGeometry)
    {
      initParams.segment(indexStartGeometries + index * sizeGeometryParameters, sizeGeometryParameters) =
          defaultGeometryData.row(defaultGeometryName.at(name)).transpose();
      index++;
    }
  }

  // Normalization coefficient
  Eigen::VectorXd coef = initParams;
  Eigen::VectorXd coefInv = initParams;
  for (size_t i = 0; i < (size_t)coef.size(); i++)
  {
    if (fabs(coef(i)) < 1e-5)
    {
      coef(i) = 0.01;
    }
    coefInv(i) = 1.0 / coef(i);
  }

  // Initial verbose
  std::cout << "Dimension: " << initParams.size() << std::endl;
  for (size_t i = 0; i < logsData.size(); i++)
  {
    std::cout << "============" << std::endl;
    std::cout << "Initial learning score for: " << filenames[i] << std::endl;
    try
    {
      scoreFitness(logsData[i], initParams, indexStartCommon, indexStartJoints, indexStartInertias,
                   indexStartGeometries, sizeJointParameters, sizeInertiaParameters, sizeGeometryParameters,
                   defaultJointData, defaultJointName, defaultInertiaData, defaultInertiaName, defaultGeometryData,
                   defaultGeometryName, 3);
    }
    catch (const std::runtime_error& e)
    {
      std::cout << "Exception: " << e.what() << std::endl;
    }
  }
  for (size_t i = 0; i < logsValidation.size(); i++)
  {
    std::cout << "============" << std::endl;
    std::cout << "Initial validation score for: " << filenamesValidation[i] << std::endl;
    try
    {
      scoreFitness(logsValidation[i], initParams, indexStartCommon, indexStartJoints, indexStartInertias,
                   indexStartGeometries, sizeJointParameters, sizeInertiaParameters, sizeGeometryParameters,
                   defaultJointData, defaultJointName, defaultInertiaData, defaultInertiaName, defaultGeometryData,
                   defaultGeometryName, 3);
    }
    catch (const std::runtime_error& e)
    {
      std::cout << "Exception: " << e.what() << std::endl;
    }
  }

  // Normalization of initial parameters
  initParams = coefInv.array() * initParams.array();
  Eigen::VectorXd bestParams = initParams;
  double bestScore = -1.0;
  int iteration = 1;

  // Fitness function
  libcmaes::FitFuncEigen fitness = [&logsData, &coef, &indexStartCommon, &indexStartJoints, &indexStartInertias,
                                    &indexStartGeometries, &sizeJointParameters, &sizeInertiaParameters,
                                    &sizeGeometryParameters, &defaultJointData, &defaultJointName, &defaultInertiaData,
                                    &defaultInertiaName, &defaultGeometryData,
                                    &defaultGeometryName](const Eigen::VectorXd& params) {
    // Check positive joint parameters
    double cost = 0.0;
    if (indexStartCommon != (size_t)-1)
    {
      for (size_t i = indexStartCommon; i < indexStartCommon + sizeJointParameters; i++)
      {
        if (params(i) * coef(i) < 0.0)
        {
          cost += 1000.0 - 1000.0 * params(i) * coef(i);
        }
      }
    }
    if (indexStartJoints != (size_t)-1)
    {
      for (size_t i = indexStartJoints; i < indexStartJoints + sizeJointParameters * namesDOFJoint.size(); i++)
      {
        if (params(i) * coef(i) < 0.0)
        {
          cost += 1000.0 - 1000.0 * params(i) * coef(i);
        }
      }
    }
    if (indexStartInertias != (size_t)-1)
    {
      for (size_t i = indexStartInertias; i < indexStartInertias + sizeInertiaParameters * namesFrameInertia.size();
           i += sizeInertiaParameters)
      {
        // 0 Mass
        if (params(i + 0) * coef(i + 0) <= 0.0)
        {
          cost += 1000.0 - 1000.0 * params(i + 0) * coef(i + 0);
        }
        // 4 Ixx
        if (params(i + 4) * coef(i + 4) <= 0.0)
        {
          cost += 1000.0 - 1000.0 * params(i + 4) * coef(i + 4);
        }
        // 7 Iyy
        if (params(i + 7) * coef(i + 7) <= 0.0)
        {
          cost += 1000.0 - 1000.0 * params(i + 7) * coef(i + 7);
        }
        // 9 Izz
        if (params(i + 9) * coef(i + 9) <= 0.0)
        {
          cost += 1000.0 - 1000.0 * params(i + 9) * coef(i + 9);
        }
      }
    }
    if (indexStartGeometries != (size_t)-1)
    {
      size_t index = 0;
      for (const std::string& name : namesFrameGeometry)
      {
        size_t tmpJ = indexStartGeometries + index * sizeGeometryParameters;
        // Penalize too large rotation correction
        // 0 Roll 1 Pitch 2 Yaw
        for (size_t k = 0; k < 3; k++)
        {
          double val = params(tmpJ + k) * coef(tmpJ + k);
          double target = defaultGeometryData(defaultGeometryName.at(name), k);
          if (fabs(val - target) > 10.0 * M_PI / 180.0)
          {
            cost += 1000.0 + 1000.0 * (fabs(val - target) - 10.0 * M_PI / 180.0);
          }
        }
        // Penalize too large translation correction
        // 3 X 4 Y 5 Z
        for (size_t k = 3; k < 6; k++)
        {
          double val = params(tmpJ + k) * coef(tmpJ + k);
          double target = defaultGeometryData(defaultGeometryName.at(name), k);
          if (fabs(val - target) > 0.005)
          {
            cost += 1000.0 + 1000.0 * (fabs(val - target) - 0.005);
          }
        }
        index++;
      }
    }
    if (cost > 0.0)
    {
      return cost;
    }
    // Iterate over on all logs
    double sumError = 0.0;
    double countError = 0.0;
    double maxError = -1.0;
    Eigen::VectorXd maxAllError(Leph::NamesDOF.size() + 2);
    for (size_t i = 0; i < Leph::NamesDOF.size() + 2; i++)
    {
      maxAllError(i) = -1.0;
    }
    for (size_t i = 0; i < logsData.size(); i++)
    {
      try
      {
        cost += scoreFitness(logsData[i], coef.array() * params.array(), indexStartCommon, indexStartJoints,
                             indexStartInertias, indexStartGeometries, sizeJointParameters, sizeInertiaParameters,
                             sizeGeometryParameters, defaultJointData, defaultJointName, defaultInertiaData,
                             defaultInertiaName, defaultGeometryData, defaultGeometryName, 0, &sumError, &countError,
                             &maxError, &maxAllError);
      }
      catch (const std::runtime_error& e)
      {
        cost += 10000.0;
      }
    }
    if (countError > 0.0 && maxError > 0.0)
    {
      return cost + 0.9 * (sumError / countError) + 0.05 * (maxAllError.mean()) + 0.05 * (maxError);
    }
    else
    {
      return cost;
    }
  };

  // Progress function
  libcmaes::ProgressFunc<libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress =
      [&bestParams, &bestScore, &iteration, &coef, &logsData, &filenames, &outputParamsFilename, &logsValidation,
       &filenamesValidation, &indexStartCommon, &indexStartJoints, &indexStartInertias, &indexStartGeometries,
       &sizeJointParameters, &sizeInertiaParameters, &sizeGeometryParameters, &defaultJointData, &defaultJointName,
       &defaultInertiaData, &defaultInertiaName, &defaultGeometryData,
       &defaultGeometryName](const libcmaes::CMAParameters<>& cmaparams, const libcmaes::CMASolutions& cmasols) {
        // Retrieve best Trajectories and score
        Eigen::VectorXd params = coef.array() * cmasols.get_best_seen_candidate().get_x_dvec().array();
        double score = cmasols.get_best_seen_candidate().get_fvalue();
        if (!std::isnan(score) && (bestScore < 0.0 || bestScore > score))
        {
          bestParams = params;
          bestScore = score;
        }
        if (iteration % 10 == 0)
        {
          // Show best optimization state
          std::cout << "============" << std::endl;
          std::cout << "Dimension: " << params.size() << std::endl;
          std::cout << "BestScore: " << bestScore << std::endl;
          // Show current optimization state
          std::cout << "Score: " << score << std::endl;
          // Saving
          saveModelParameters(outputParamsFilename, bestParams, indexStartCommon, indexStartJoints, indexStartInertias,
                              indexStartGeometries, sizeJointParameters, sizeInertiaParameters, sizeGeometryParameters,
                              defaultJointData, defaultJointName, defaultInertiaData, defaultInertiaName,
                              defaultGeometryData, defaultGeometryName);
          std::cout << "Best parameters dumped to: " << outputParamsFilename << std::endl;
          std::cout << "============" << std::endl;
          std::cout << "Learning data:" << std::endl;
          for (size_t i = 0; i < logsData.size(); i++)
          {
            std::cout << "==== Best score for: " << filenames[i] << std::endl;
            scoreFitness(logsData[i], bestParams, indexStartCommon, indexStartJoints, indexStartInertias,
                         indexStartGeometries, sizeJointParameters, sizeInertiaParameters, sizeGeometryParameters,
                         defaultJointData, defaultJointName, defaultInertiaData, defaultInertiaName,
                         defaultGeometryData, defaultGeometryName, 1);
          }
          std::cout << "============" << std::endl;
          std::cout << "Validation data:" << std::endl;
          for (size_t i = 0; i < logsValidation.size(); i++)
          {
            std::cout << "==== Best score for: " << filenamesValidation[i] << std::endl;
            scoreFitness(logsValidation[i], bestParams, indexStartCommon, indexStartJoints, indexStartInertias,
                         indexStartGeometries, sizeJointParameters, sizeInertiaParameters, sizeGeometryParameters,
                         defaultJointData, defaultJointName, defaultInertiaData, defaultInertiaName,
                         defaultGeometryData, defaultGeometryName, 1);
          }
          std::cout << "============" << std::endl;
        }
        iteration++;

        // Call default CMA-ES default progress function
        return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>::_defaultPFunc(cmaparams, cmasols);
      };

  // CMAES initialization
  libcmaes::CMAParameters<> cmaparams(initParams, cmaesSigma, cmaesLambda);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(true);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_elitism(cmaesElitismLevel);
  cmaparams.set_restarts(cmaesRestarts);
  cmaparams.set_max_iter(cmaesMaxIterations);
  cmaparams.set_ftolerance(1e-9);

  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams, progress);

  // Retrieve best Trajectories and score
  bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
  bestScore = cmasols.get_best_seen_candidate().get_fvalue();

  // Final verbose
  for (size_t i = 0; i < logsData.size(); i++)
  {
    std::cout << "Final score for: " << filenames[i] << std::endl;
    scoreFitness(logsData[i], bestParams.array(), indexStartCommon, indexStartJoints, indexStartInertias,
                 indexStartGeometries, sizeJointParameters, sizeInertiaParameters, sizeGeometryParameters,
                 defaultJointData, defaultJointName, defaultInertiaData, defaultInertiaName, defaultGeometryData,
                 defaultGeometryName, 3);
  }
  std::cout << "BestParams: " << bestParams.transpose() << std::endl;
  std::cout << "BestScore: " << bestScore << std::endl;

  return 0;
}
