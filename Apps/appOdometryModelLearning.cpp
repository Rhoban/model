#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "Model/HumanoidFixedModel.hpp"
#include "Odometry/Odometry.hpp"
#include "Calibration/LogLikelihoodMaximization.hpp"
#include "Utils/FileEigen.h"
#include "Plot/Plot.hpp"

/**
 * Odometry computation type
 */
enum OdometryType
{
  // No model. Use walk order as input
  OdometryOrder,
  // Use goal model state as input
  OdometryGoal,
  // Use read model state as input
  OdometryRead,
};

/**
 * Number of sampling per parameters evaluation
 * for log likelyhood maximisation
 */
static const unsigned int samplingNumber = 200;

/**
 * Learning/Testing data ratio between 0 and 1
 */
static const double learningDataRatio = 0.75;

/**
 * CMA-ES optimization configuration
 */
static const int cmaesElitismLevel = 0;
static const unsigned int cmaesMaxIterations = 100;
static const unsigned int cmaesRestarts = 1;
static const unsigned int cmaesLambda = 10;
static const double cmaesSigma = -1.0;

/**
 * Used odometry displacement
 * and noise models
 */
static Leph::OdometryDisplacementModel::Type typeDisplacement =
    Leph::OdometryDisplacementModel::DisplacementProportionalXYA;
static Leph::OdometryNoiseModel::Type typeNoise = Leph::OdometryNoiseModel::NoiseConstant;

/**
 * Used odometry mode
 */
static OdometryType typeOdometry = OdometryOrder;

/**
 * Odometry data for one
 * recorded sequence
 */
struct OdometrySequence
{
  // Model read and goal model cartesian
  // pose in origin at each logged point
  //(X,Y,Theta).
  std::vector<Eigen::Vector3d> readTrajsPose;
  std::vector<Leph::HumanoidFixedModel::SupportFoot> readTrajsSupport;
  std::vector<Eigen::Vector3d> goalTrajsPose;
  std::vector<Leph::HumanoidFixedModel::SupportFoot> goalTrajsSupport;
  // Walk step, lateral, turn and phase
  // walk order at each logged point
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> walkTrajsOrder;
  std::vector<double> walkTrajsPhase;
  // Target observed cartesian
  // displacement  (X,Y,Theta)
  Eigen::Vector3d targetDisplacements;
};

/**
 * Append data loaded from fiven filename to
 * given OdometrySequence container
 */
static void loadOdometryDataFromFile(std::vector<OdometrySequence>& data, const std::string& filename)
{
  // Open file
  std::ifstream file(filename);
  if (!file.is_open())
  {
    throw std::runtime_error("Unable to open log file: " + filename);
  }

  // Loop over file entries
  size_t lastSeq = -1;
  while (file.good() && file.peek() != EOF)
  {
    // Skip end of line
    while (file.peek() == ' ' || file.peek() == '\n')
    {
      file.ignore();
    }
    if (!file.good() || file.peek() == EOF)
    {
      break;
    }
    // Retrieve one file line
    size_t seq;
    size_t index;
    double readPoseX;
    double readPoseY;
    double readPoseYaw;
    int readSupportFoot;
    double goalPoseX;
    double goalPoseY;
    double goalPoseYaw;
    int goalSupportFoot;
    double walkOrderX;
    double walkOrderY;
    double walkOrderTheta;
    double walkOrderEnabled;
    double walkPhase;
    double targetX;
    double targetY;
    double targetA;
    file >> seq;
    file >> index;
    file >> readPoseX;
    file >> readPoseY;
    file >> readPoseYaw;
    file >> readSupportFoot;
    file >> goalPoseX;
    file >> goalPoseY;
    file >> goalPoseYaw;
    file >> goalSupportFoot;
    file >> walkOrderX;
    file >> walkOrderY;
    file >> walkOrderTheta;
    file >> walkOrderEnabled;
    file >> walkPhase;
    file >> targetX;
    file >> targetY;
    file >> targetA;
    if (lastSeq != seq)
    {
      // Start a new sequence
      data.push_back(OdometrySequence());
      data.back().targetDisplacements = Eigen::Vector3d(targetX, targetY, targetA);
    }
    lastSeq = seq;
    data.back().readTrajsPose.push_back(Eigen::Vector3d(readPoseX, readPoseY, readPoseYaw));
    data.back().readTrajsSupport.push_back((Leph::HumanoidFixedModel::SupportFoot)readSupportFoot);
    data.back().goalTrajsPose.push_back(Eigen::Vector3d(goalPoseX, goalPoseY, goalPoseYaw));
    data.back().goalTrajsSupport.push_back((Leph::HumanoidFixedModel::SupportFoot)goalSupportFoot);
    data.back().walkTrajsOrder.push_back(Eigen::Vector4d(walkOrderX, walkOrderY, walkOrderTheta, walkOrderEnabled));
    data.back().walkTrajsPhase.push_back(walkPhase);
  }
  file.close();
}

/**
 * Build and return the initial
 * parameters vector
 */
Eigen::VectorXd buildInitialParameters()
{
  Leph::Odometry odometry(typeDisplacement, typeNoise);
  return odometry.getParameters();
}

/**
 * Build and return the
 * normalization coefficients
 */
Eigen::VectorXd buildNormalizationCoef()
{
  Leph::Odometry odometry(typeDisplacement, typeNoise);
  return odometry.getNormalization();
}

/**
 * Check and bound parameters to
 * acceptable range.
 * Return non zero cost if
 * parameters are unbounded.
 */
double boundParameters(const Eigen::VectorXd& params)
{
  Leph::Odometry odometry(typeDisplacement, typeNoise);
  double cost = odometry.setParameters(params);
  if (cost > 0.0)
  {
    return 1000.0 + 1000.0 * cost;
  }
  else
  {
    return 0.0;
  }
}

/**
 * Model initialization function
 */
Leph::Odometry initModel(const Eigen::VectorXd& params)
{
  Leph::Odometry odometry(typeDisplacement, typeNoise);
  double cost = odometry.setParameters(params);
  if (cost > 0.0)
  {
    std::cout << "Parameters: " << params.transpose() << std::endl;
    throw std::logic_error("Parameters out of bounds");
  }

  return odometry;
}

/**
 * Compute and return the estimation
 * (try to predict observation) from
 * given data and using given parameters
 */
Eigen::VectorXd evaluateParametersPlot(const Eigen::VectorXd& params, const OdometrySequence& data, bool noRandom,
                                       Leph::Odometry& odometry, std::default_random_engine& engine, Leph::Plot* plot)
{
  (void)params;

  odometry.reset();

  std::default_random_engine* usedEngine = &engine;
  if (noRandom)
  {
    // Disable the random engien to disable
    // the noise model in odometry
    usedEngine = nullptr;
  }

  // Iterate over recorded point inside the sequence
  double lastPhase = data.walkTrajsPhase.front();
  for (size_t i = 0; i < data.readTrajsPose.size(); i++)
  {
    double phase = data.walkTrajsPhase[i];
    // Use given odometry type
    if (typeOdometry == OdometryOrder)
    {
      if (lastPhase > 0.8 && phase < 0.2)
      {
        double enableGain = data.walkTrajsOrder[i](3);
        odometry.updateFullStep(2.0 * data.walkTrajsOrder[i].segment(0, 3) * enableGain, usedEngine);
      }
    }
    else if (typeOdometry == OdometryGoal)
    {
      odometry.update(data.goalTrajsPose[i], data.goalTrajsSupport[i], usedEngine);
    }
    else if (typeOdometry == OdometryRead)
    {
      odometry.update(data.readTrajsPose[i], data.readTrajsSupport[i], usedEngine);
    }
    else
    {
      throw std::logic_error("Invalid odometry type");
    }
    lastPhase = phase;
    if (plot != nullptr)
    {
      plot->add({
          "state_x",
          odometry.state().x(),
          "state_y",
          odometry.state().y(),
          "state_z",
          odometry.state().z(),
          "seq",
          i,
      });
    }
  }

  // Final integrated state
  Eigen::VectorXd estimate(3);
  estimate << odometry.state().x(), odometry.state().y(), odometry.state().z();
  return estimate;
}
Eigen::VectorXd evaluateParameters(const Eigen::VectorXd& params, const OdometrySequence& data, bool noRandom,
                                   Leph::Odometry& odometry, std::default_random_engine& engine)
{
  return evaluateParametersPlot(params, data, noRandom, odometry, engine, nullptr);
}

/**
 * Simulate and plot given
 * odometry sequence using given parameters
 */
static void displaySequences(const std::vector<OdometrySequence>& logs, const Eigen::VectorXd& params)
{
  Leph::Odometry tmpOdometry = initModel(params);
  std::random_device rd;
  std::default_random_engine tmpEngine(rd());

  for (size_t i = 0; i < logs.size(); i++)
  {
    std::cout << "Seq " << i << " size=" << logs[i].readTrajsPose.size() << std::endl;
    Leph::Plot plot;
    // Noise less run
    Eigen::VectorXd estimate = evaluateParametersPlot(params, logs[i], true, tmpOdometry, tmpEngine, &plot);
    // Noise runs
    for (int k = 0; k < 1000; k++)
    {
      Eigen::VectorXd estimate = evaluateParametersPlot(params, logs[i], false, tmpOdometry, tmpEngine, nullptr);
      plot.add({
          "sample_x",
          estimate.x(),
          "sample_y",
          estimate.y(),
          "sample_z",
          estimate.z(),
      });
    }
    plot.add({
        "obs_x",
        logs[i].targetDisplacements.x(),
        "obs_y",
        logs[i].targetDisplacements.y(),
        "obs_z",
        Leph::AngleBound(-logs[i].targetDisplacements.z() * 2.0 * M_PI / 12.0),
    });
    plot.rangeUniform(true)
        .plot("state_x", "state_y", Leph::Plot::LinesPoints, "seq")
        .plot("sample_x", "sample_y", Leph::Plot::Points)
        .plot("obs_x", "obs_y", Leph::Plot::Points)
        .render();
    plot.plot("index", "state_z", Leph::Plot::LinesPoints, "seq")
        .plot("index", "sample_z", Leph::Plot::Points)
        .plot("index", "obs_z", Leph::Plot::Points)
        .render();
  }
}

/**
 * Calibrate the Odometry correction and noise
 * parameters using log likelihood maximisation
 * through CMA-ES optimization.
 */
int main(int argc, char** argv)
{
  if (argc != 3 && argc != 5)
  {
    std::cout << "Usage: ./app "
              << "odometry.data outputPrefix [SEED seed.odometryparams]" << std::endl;
    return 1;
  }
  std::string logPath = argv[1];
  std::string outPath = argv[2];
  std::string seedPath = "";
  if (argc == 5 && std::string(argv[3]) == "SEED")
  {
    seedPath = argv[4];
  }

  // Loading log sequences
  std::vector<OdometrySequence> logs;
  loadOdometryDataFromFile(logs, logPath);
  std::cout << "Loading odometry data from: " << logPath << " with " << logs.size() << " sequences" << std::endl;

  // Parameters initialization
  Eigen::VectorXd initParams = buildInitialParameters();

  // Load seeded parameters
  if (seedPath != "")
  {
    std::cout << "Loading parameters from: " << seedPath << std::endl;
    // Open file
    std::ifstream file(seedPath);
    if (!file.is_open())
    {
      throw std::runtime_error("Unable to open file: " + seedPath);
    }
    // Read data
    int tmpTypeDisplacement;
    int tmpTypeNoise;
    file >> tmpTypeDisplacement;
    file >> tmpTypeNoise;
    typeDisplacement = (Leph::OdometryDisplacementModel::Type)tmpTypeDisplacement;
    typeNoise = (Leph::OdometryNoiseModel::Type)tmpTypeNoise;
    initParams = Leph::ReadEigenVectorFromStream(file);
    file.close();
    // Check bounds
    Leph::Odometry tmpOdometry(typeDisplacement, typeNoise);
    double isError = tmpOdometry.setParameters(initParams);
    if (isError > 0.0)
    {
      std::cout << "Seed parameters out of bounds: " << isError << std::endl;
    }
  }

  // Initialize the logLikelihood
  // maximisation process
  Leph::LogLikelihoodMaximization<OdometrySequence, Leph::Odometry> calibration;
  calibration.setInitialParameters(initParams, buildNormalizationCoef());
  calibration.setUserFunctions(evaluateParameters, boundParameters, initModel,
                               [](const Eigen::VectorXd& vect) -> Eigen::VectorXd {
                                 // For estimation and observation
                                 // direct comparison, use only
                                 // cartesian distance
                                 return vect.segment(0, 2);
                               });
  // Add observations data
  for (size_t i = 0; i < logs.size(); i++)
  {
    Eigen::VectorXd obs(3);
    obs << logs[i].targetDisplacements.x(), logs[i].targetDisplacements.y(),
        Leph::AngleBound(-logs[i].targetDisplacements.z() * 2.0 * M_PI / 12.0);
    calibration.addObservation(obs, logs[i]);
  }

  // Start the CMA-ES optimization
  Leph::Plot plot;
  calibration.runOptimization(samplingNumber, learningDataRatio, cmaesMaxIterations, cmaesRestarts, cmaesLambda,
                              cmaesSigma, cmaesElitismLevel, &plot);

  // Display the best found parameters
  Eigen::VectorXd bestParams = calibration.getParameters();
  Leph::Odometry tmpOdometry = initModel(bestParams);
  tmpOdometry.printParameters();
  // Export the optimized model parameters
  // Write to file
  std::string odoParamsPath = outPath + "odometryModel.params";
  std::cout << "Writing odometry parameters to: " << odoParamsPath << std::endl;
  std::ofstream file(odoParamsPath);
  if (!file.is_open())
  {
    throw std::runtime_error("Unable to open file: " + odoParamsPath);
  }
  file << (int)tmpOdometry.getDisplacementType() << " " << (int)tmpOdometry.getNoiseType() << std::endl;
  Leph::WriteEigenVectorToStream(file, bestParams);
  file.close();

  // Plot the convergence graphic
  // plot.plot("iteration", "all").render();
  // Plot the sequences with best parameters
  displaySequences(logs, bestParams);

  return 0;
}
