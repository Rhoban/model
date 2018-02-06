#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "Model/HumanoidFixedModel.hpp"
#include "Odometry/Odometry.hpp"
#include "Odometry/OdometrySequence.hpp"
#include "Calibration/LogLikelihoodMaximization.hpp"
#include "Utils/Angle.h"
#include "Utils/FileEigen.h"

/**
 *
 */
enum RunType {
  Diagnose, // Try to identify which logs are likely to be 'wrong'
  Classic,  // Simple calibration run on the data
  Ascending // Increase the size of the learning_set
};

/**
 * How is position evaluated
 */
enum PosEvalType {
  Cartesian,
  Polar,
  None
};

// The whole configuration of the learner is included here
struct LearningConfig{
  RunType run_type;
  int nb_replicates;// How much replicate of the run are used?
  PosEvalType pos_eval_type;
  bool evaluate_angle;
  Leph::OdometryType odometry_type;
  Leph::OdometryDisplacementModel::Type displacement_type;
  Leph::OdometryNoiseModel::Type noise_type;
  unsigned int sampling_number;
  unsigned int learning_set_size;
  int cmaes_elitism;
  int cmaes_max_iterations;
  int cmaes_restarts;
  int cmaes_lambda;
  int cmaes_sigma;
};

struct LearningConfig conf;

// Bounds for cartesian noise of the localisation process (direction is random)
double minLocCartNoise = std::pow(10,-3);
double maxLocCartNoise = 1;
// Bounds for angular noise of the localisation process [rad]
double minLocAngNoise = 0.1 * M_PI / 180;
double maxLocAngNoise = 20 * M_PI / 180;

// Noise parameters: devPos [m], devTheta [rad]
// Result: pos_x, pos_y, pos_theta
Eigen::Vector3d sampleObservationNoise(const Eigen::VectorXd & noiseParameters,
                                       std::default_random_engine * engine)
{
  double devPos = noiseParameters(0);
  double devTheta = noiseParameters(1);
  // Generatic noise
  double posErrNorm, posErrArg, thetaErr;
  posErrNorm = std::normal_distribution<double>(0.0, devPos)(*engine);
  posErrArg  = std::uniform_real_distribution<double>(0, 2*M_PI)(*engine);
  thetaErr   = std::normal_distribution<double>(0.0, devTheta)(*engine);
  // Generating noise
  return Eigen::Vector3d(posErrNorm * cos(posErrArg),
                         posErrArg * sin(posErrArg),
                         Leph::AngleBound(thetaErr));

}

Eigen::VectorXd buildObservation(const Eigen::Vector3d & final_state,
                                 LearningConfig * conf)
{
  int output_dim = 0;
  if (conf->pos_eval_type != PosEvalType::None) output_dim += 2;
  if (conf->evaluate_angle) output_dim++;
  if (output_dim == 0) {
    throw std::logic_error("Conf should eval at least pos or angle");
  }
  Eigen::VectorXd observation(output_dim);
  int index = 0;
  switch(conf->pos_eval_type) {
    case PosEvalType::None: break;
    case PosEvalType::Cartesian:
      observation(index) = final_state(0);
      observation(index+1) = final_state(1);
      index += 2;
      break;
    case PosEvalType::Polar:
    {
      double x = final_state(0);
      double y = final_state(1);
      double dist = std::sqrt(x*x + y*y);
      double theta = std::atan2(y,x);
      observation(index) = dist;
      observation(index+1) = theta;
      index += 2;
      break;
    }
  }
  if (conf->evaluate_angle) {
    observation(index) = final_state(2);
    index++;
  }
  return observation;
}

Eigen::VectorXi getCyclicDims(LearningConfig * conf)
{
  int output_dim = 0;
  if (conf->pos_eval_type != PosEvalType::None) output_dim += 2;
  if (conf->evaluate_angle) output_dim++;
  if (output_dim == 0) {
    throw std::logic_error("Conf should eval at least pos or angle");
  }
  Eigen::VectorXi result(output_dim);
  int index = 0;
  switch(conf->pos_eval_type) {
    case PosEvalType::None: break;
    case PosEvalType::Cartesian:
      result(index) = 0;
      result(index+1) = 0;
      index += 2;
      break;
    case PosEvalType::Polar:
    {
      result(index)   = 0;
      result(index+1) = 1;
      index += 2;
      break;
    }
  }
  if (conf->evaluate_angle) {
    result(index) = 1;
    index++;
  }
  return result;
}

// Read next line as a RunType
RunType parseRunType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "Diagnose" ) return RunType::Diagnose;
  if (line == "Classic"  ) return RunType::Classic;
  if (line == "Ascending") return RunType::Ascending;
  throw std::runtime_error("Unknown type of run: '" + line + "'");
}

// Read next line as a PosEvalType
PosEvalType parsePosEvalType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "Cartesian") return PosEvalType::Cartesian;
  if (line == "Polar")     return PosEvalType::Polar;
  if (line == "None")      return PosEvalType::None;
  throw std::runtime_error("Unknown type of position evaluation: '" + line + "'");
}

// Read next line as an OdometryType
Leph::OdometryType parseOdometryType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "OdometryOrder") return Leph::OdometryType::OdometryOrder;
  if (line == "OdometryGoal")  return Leph::OdometryType::OdometryGoal;
  if (line == "OdometryRead")  return Leph::OdometryType::OdometryRead;
  throw std::runtime_error("Unknown type of odometry: '" + line + "'");
}

// Should be in OdometryDisplacementModel
Leph::OdometryDisplacementModel::Type parseDisplacementType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "ProportionalXY" ) return Leph::OdometryDisplacementModel::Type::DisplacementProportionalXY;
  if (line == "ProportionalXYA") return Leph::OdometryDisplacementModel::Type::DisplacementProportionalXYA;
  if (line == "LinearSimpleXY" ) return Leph::OdometryDisplacementModel::Type::DisplacementLinearSimpleXY;
  if (line == "LinearSimpleXYA") return Leph::OdometryDisplacementModel::Type::DisplacementLinearSimpleXYA;
  if (line == "LinearFullXY"   ) return Leph::OdometryDisplacementModel::Type::DisplacementLinearFullXY;
  if (line == "LinearFullXYA"  ) return Leph::OdometryDisplacementModel::Type::DisplacementLinearFullXYA;
  throw std::runtime_error("Unknown type of displacement Model: '" + line + "'");
}

// Should be in OdometryNoiseModel
Leph::OdometryNoiseModel::Type parseNoiseType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "Constant"    ) return Leph::OdometryNoiseModel::Type::NoiseConstant;
  if (line == "Proportional") return Leph::OdometryNoiseModel::Type::NoiseProportional;
  if (line == "LinearSimple") return Leph::OdometryNoiseModel::Type::NoiseLinearSimple;
  if (line == "LinearFull"  ) return Leph::OdometryNoiseModel::Type::NoiseLinearFull;
  throw std::runtime_error("Unknown type of noise Model: '" + line + "'");
}

// Read next line as a bool
bool parseBool(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "true")  return true;
  if (line == "false") return false;
  throw std::runtime_error("Invalid line for bool: '" + line + "'");
}

// Read next line as an int
int parseInt(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  try {
    return std::stoi(line);
  } catch(const std::invalid_argument & exc) {
    throw std::runtime_error("Failed to parse '" + line + "' as an int");
  }
}

/// Dirty code: read config
void readLearningConfig(const std::string & filename,
                        struct LearningConfig * conf)
{
  std::ifstream input(filename);
  conf->run_type = parseRunType(input);
  conf->nb_replicates = parseInt(input);
  conf->pos_eval_type = parsePosEvalType(input);
  conf->evaluate_angle = parseBool(input);
  conf->odometry_type = parseOdometryType(input);
  conf->displacement_type = parseDisplacementType(input);
  conf->noise_type = parseNoiseType(input);
  // TODO: produces silent error on invalid content, should be fixed
  input >> conf->sampling_number
        >> conf->learning_set_size
        >> conf->cmaes_elitism
        >> conf->cmaes_max_iterations
        >> conf->cmaes_restarts
        >> conf->cmaes_lambda
        >> conf->cmaes_sigma;
  input.close();
}

void writeLearningConfig(std::ostream & out, struct LearningConfig * conf)
{
  out << "PosEvalType     : " << conf->pos_eval_type        << std::endl
      << "EvaluateAngle   : " << conf->evaluate_angle       << std::endl
      << "OdometryType    : " << conf->odometry_type        << std::endl
      << "DisplacementType: " << conf->displacement_type    << std::endl
      << "NoiseType       : " << conf->noise_type           << std::endl
      << "SamplingNumber  : " << conf->sampling_number      << std::endl
      << "LearningSetSize : " << conf->learning_set_size    << std::endl
      << "Elitism         : " << conf->cmaes_elitism        << std::endl
      << "MaxIterations   : " << conf->cmaes_max_iterations << std::endl
      << "Restarts        : " << conf->cmaes_restarts       << std::endl
      << "Lambda          : " << conf->cmaes_lambda         << std::endl
      << "Sigma           : " << conf->cmaes_sigma          << std::endl;
}

/**
 * Build and return the initial 
 * parameters vector
 */
Eigen::VectorXd buildInitialParameters()
{
  Leph::Odometry odometry(conf.displacement_type, conf.noise_type);
  Eigen::VectorXd odomParameters = odometry.getParameters();
  Eigen::VectorXd parameters(odomParameters.rows() + 2);
  parameters(0) = 0.01;
  parameters(1) = 5 * M_PI / 180;
  parameters.segment(2, odomParameters.rows()) = odomParameters;
  return parameters;
}

/**
 * Build and return the 
 * normalization coefficients
 */
Eigen::VectorXd buildNormalizationCoef()
{
  Leph::Odometry odometry(conf.displacement_type, conf.noise_type);
  Eigen::VectorXd odomCoeffs = odometry.getNormalization();
  Eigen::VectorXd coeffs(odomCoeffs.rows() + 2);
  coeffs(0) = maxLocCartNoise;
  coeffs(1) = maxLocAngNoise;
  coeffs.segment(2, odomCoeffs.rows()) = odomCoeffs;
  return coeffs;
}

/**
 * Check and bound parameters to
 * acceptable range.
 * Return non zero cost if
 * parameters are unbounded.
 */
double boundParameters(
  const Eigen::VectorXd& params)
{
  // Creating vectors for easy comparison
  Eigen::Vector2d minLocNoise(minLocCartNoise, minLocAngNoise);
  Eigen::Vector2d maxLocNoise(maxLocCartNoise, maxLocAngNoise);
  // Cost for Localisation noises
  double cost = 0;
  for (int dim=0;dim<minLocNoise.rows();dim++) {
    if (params(dim) < minLocNoise(dim)) {
      cost += minLocNoise(dim) - params(dim);
    }
    if (params(dim) > maxLocNoise(dim)) {
      cost += params(dim) - maxLocNoise(dim);
    }
  }
  // Cost for odometry parameters
  Leph::Odometry odometry(conf.displacement_type, conf.noise_type);
  cost += odometry.setParameters(params.segment(2, params.rows() -2));
  if (cost > 0.0) {
    return 1000.0 + 1000.0*cost;
  } else {
    return 0.0;
  }
}

/**
 * Model initialization function
 */
Leph::Odometry initModel(const Eigen::VectorXd& params)
{
  Leph::Odometry odometry(conf.displacement_type, conf.noise_type);
  double cost = odometry.setParameters(params.segment(2, params.rows()-2));
  if (cost > 0.0) {
    std::cout << "Parameters: " 
              << params.transpose() << std::endl;
    throw std::logic_error("Parameters out of bounds");
  }

  return odometry;
}

/**
 * Compute and return the estimation
 * (try to predict observation) from
 * given data and using given parameters
 */
Eigen::VectorXd evaluateParameters(
  const Eigen::VectorXd& params,
  const Leph::OdometrySequence& data,
  bool noRandom,
  Leph::Odometry& odometry,
  std::default_random_engine& engine)
{
  (void)params;

  simulateOdometry(data, noRandom, odometry, conf.odometry_type, engine,
                   nullptr);

  Eigen::Vector3d initialObsNoise = sampleObservationNoise(params.segment(0,2), &engine);
  Eigen::Vector3d finalObsNoise   = sampleObservationNoise(params.segment(0,2), &engine);

  Eigen::Vector3d state(0,0,0);
  odometry.odometryInt(initialObsNoise, state);
  odometry.odometryInt(odometry.state(), state);
  odometry.odometryInt(finalObsNoise, state);


  //Final integrated state
  return buildObservation(state, &conf);
}


void writeResultHeader(std::ostream & out)
{
  out << "learningSize,testLog,displacementModel,"
      << "noiseModel,learningScore,testScore" << std::endl;
}

void writeResultLine(std::ostream & out,
                     double learningScore,
                     double validationScore,
                     size_t * logId)
{
  if (logId != nullptr) {
    out << "NA," << (*logId) << ",";
  } else {
    out << conf.learning_set_size << ",NA,";
  }
  out << conf.displacement_type << ","
      << conf.noise_type << ","
      << learningScore << ","
      << validationScore << std::endl;
}

void writeParamsHeader(std::ostream & out)
{
  out << "learningSize,testLog,displacementModel,noiseModel,loc_cart_noise,loc_angle_noise,";
  // Getting number of odometry parameters
  Leph::Odometry o(conf.displacement_type, conf.noise_type);
  std::vector<std::string> names = o.getParametersNames();
  for (size_t i = 0; i < names.size(); i++) {
    out << names[i];
    if (i < names.size() - 1) out << ",";
  }
  out << std::endl;
}

void writeParamsLine(std::ostream & out,
                     const Eigen::VectorXd & params,
                     size_t * logId)
{
  if (logId != nullptr) {
    out << "NA," << (*logId) << ",";
  } else {
    out << conf.learning_set_size << ",NA,";
  }
  out << conf.displacement_type << "," << conf.noise_type << ",";
  for (int i = 0; i < params.rows(); i++) {
    out << params(i);
    if (i < params.rows() - 1) out << ",";
  }
  out << std::endl;
}

void runLearning(const std::vector<Leph::OdometrySequence> & logs,
                 std::ostream & result_output,
                 std::ostream & params_output,
                 size_t * logId)
{
  //Parameters initialization
  Eigen::VectorXd initParams = buildInitialParameters();
      
  //Initialize the logLikelihood 
  //maximisation process
  Leph::LogLikelihoodMaximization
    <Leph::OdometrySequence, Leph::Odometry> calibration;
  calibration.setCyclicObsDims(getCyclicDims(&conf));
  calibration.setInitialParameters(
    initParams, buildNormalizationCoef());
  calibration.setUserFunctions(evaluateParameters, boundParameters, initModel);
  //Add observations data
  for (size_t i=0;i<logs.size();i++) {
    Eigen::Vector3d final_state = logs[i].targetDisplacements;
    calibration.addObservation(buildObservation(final_state, &conf), logs[i]);
  }

  // If learningSet have been specified use it
  if (logId != nullptr) {
    // Build validation sets
    std::vector<size_t> learningSet;
    std::vector<size_t> validationSet = {*logId};
    for (size_t i = 0; i < logs.size(); i++) {
      if (i != *logId) learningSet.push_back(i);
    }
    calibration.runOptimization(conf.sampling_number,
                                learningSet,
                                validationSet,
                                conf.cmaes_max_iterations, 
                                conf.cmaes_restarts, 
                                conf.cmaes_lambda, 
                                conf.cmaes_sigma, 
                                conf.cmaes_elitism,
                                nullptr);
  }
  else {
    calibration.runOptimization(conf.sampling_number,
                                conf.learning_set_size,
                                conf.cmaes_max_iterations, 
                                conf.cmaes_restarts, 
                                conf.cmaes_lambda, 
                                conf.cmaes_sigma, 
                                conf.cmaes_elitism,
                                nullptr);
  }

  // Get calibrationResult
  Eigen::VectorXd bestParams = calibration.getParameters();
  double learningScore = calibration.scoreFitness(bestParams, false);
  double validationScore = calibration.scoreFitness(bestParams, true);
  writeResultLine(result_output, learningScore, validationScore, logId);
  writeParamsLine(params_output, bestParams, logId);
}

void diagnose(const std::vector<Leph::OdometrySequence> & logs,
              std::ostream & result_output,
              std::ostream & params_output)
{
  for (size_t logId = 0; logId < logs.size(); logId++) {
    runLearning(logs, result_output, params_output, &logId);
  }
}

void classic(const std::vector<Leph::OdometrySequence> & logs,
             std::ostream & result_output,
             std::ostream & params_output)
{
  runLearning(logs, result_output, params_output, nullptr);
}

void ascending(const std::vector<Leph::OdometrySequence> & logs,
               std::ostream & result_output,
               std::ostream & params_output)
{
  int max_learning_set_size = conf.learning_set_size;
  for (int i=1; i <= max_learning_set_size; i++) {
    conf.learning_set_size = i;       
    runLearning(logs, result_output, params_output, nullptr);
  }
  // Restore old value
  conf.learning_set_size = max_learning_set_size;
}

/**
 * Calibrate the Odometry correction and noise 
 * parameters using log likelihood maximisation 
 * through CMA-ES optimization.
 */
int main(int argc, char** argv)
{
  if (argc != 4) {
    std::cout << "Usage: ./app "
              << "odometry.data learningConfig.conf outputPrefix" 
              << std::endl;
    return 1;
  }
  std::string logPath = argv[1];
  std::string confPath = argv[2];
  std::string outPath = argv[3];

  //Loading config
  readLearningConfig(confPath, &conf);
  writeLearningConfig(std::cout, &conf);
    
  //Loading log sequences
  std::vector<Leph::OdometrySequence> logs;
  loadOdometryDataFromFile(logs, logPath);
  std::cout << "Loading odometry data from: " 
            << logPath << " with " 
            << logs.size()
            << " sequences" << std::endl;

  std::ofstream result_out(outPath + "results.csv");
  std::ofstream params_out(outPath + "params.csv");
  writeResultHeader(result_out);
  writeParamsHeader(params_out);

  for (int i = 0; i < conf.nb_replicates; i++) {
    switch(conf.run_type) {
      case RunType::Diagnose:
        diagnose(logs,result_out,params_out);
        break;
      case RunType::Classic:
        classic(logs,result_out,params_out);
        break;
      case RunType::Ascending:
        ascending(logs,result_out,params_out);
        break;
    }
  }

  return 0;
}

