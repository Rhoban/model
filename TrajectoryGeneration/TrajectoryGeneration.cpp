#include <stdexcept>
#include <libcmaes/cmaes.h>
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "Utils/FileModelParameters.h"
#include "Utils/time.h"
#include "Model/NamesModel.h"
#include "Utils/AxisAngle.h"

namespace Leph
{
TrajectoryGeneration::TrajectoryGeneration(RobotType type, const std::string& modelParamsPath)
  : _type(type)
  , _modelParametersPath(modelParamsPath)
  , _initialParameters()
  , _normCoefs()
  , _generateFunc()
  , _checkParamsFunc()
  , _checkStateFunc()
  , _checkDOFFunc()
  , _scoreFunc()
  , _endScoreFunc()
  , _scoreSimFunc()
  , _endScoreSimFunc()
  , _saveFunc()
  , _bestTraj()
  , _bestParams()
  , _bestScore(0.0)
  , _countIteration(1)
{
}

void TrajectoryGeneration::setInitialParameters(const Eigen::VectorXd& params)
{
  _initialParameters = params;
}

void TrajectoryGeneration::setNormalizationCoefs(const Eigen::VectorXd& normCoefs)
{
  if (normCoefs.size() != _initialParameters.size())
  {
    throw std::logic_error("TrajectoryGeneration invalid norm coefs size: " + std::to_string(normCoefs.size()));
  }
  _normCoefs = normCoefs;
}

void TrajectoryGeneration::setTrajectoryGenerationFunc(GenerationFunc func)
{
  _generateFunc = func;
}

void TrajectoryGeneration::setCheckParametersFunc(CheckParamsFunc func)
{
  _checkParamsFunc = func;
}

void TrajectoryGeneration::setCheckStateFunc(CheckStateFunc func)
{
  _checkStateFunc = func;
}

void TrajectoryGeneration::setCheckDOFFunc(CheckDOFFunc func)
{
  _checkDOFFunc = func;
}

void TrajectoryGeneration::setScoreFunc(ScoreFunc func)
{
  _scoreFunc = func;
}

void TrajectoryGeneration::setEndScoreFunc(EndScoreFunc func)
{
  _endScoreFunc = func;
}

void TrajectoryGeneration::setScoreSimFunc(ScoreSimFunc func)
{
  _scoreSimFunc = func;
}

void TrajectoryGeneration::setEndScoreSimFunc(EndScoreSimFunc func)
{
  _endScoreSimFunc = func;
}

void TrajectoryGeneration::setSaveFunc(SaveFunc func)
{
  _saveFunc = func;
}

Eigen::VectorXd TrajectoryGeneration::initialParameters() const
{
  return _initialParameters;
}

Eigen::VectorXd TrajectoryGeneration::normalizationCoefs() const
{
  if (_normCoefs.size() == 0)
  {
    return Eigen::VectorXd::Ones(_initialParameters.size());
  }
  else
  {
    return _normCoefs;
  }
}

const std::string& TrajectoryGeneration::modelParametersPath() const
{
  return _modelParametersPath;
}

Trajectories TrajectoryGeneration::generateTrajectory(const Eigen::VectorXd& params) const
{
  return _generateFunc(params);
}

double TrajectoryGeneration::checkParameters(const Eigen::VectorXd& params) const
{
  return _checkParamsFunc(params);
}
double TrajectoryGeneration::checkState(const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                                        const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                                        const Eigen::Vector3d& footAxis) const
{
  return _checkStateFunc(params, t, trunkPos, trunkAxis, footPos, footAxis);
}
double TrajectoryGeneration::checkDOF(const Eigen::VectorXd& params, double t, const HumanoidModel& model) const
{
  return _checkDOFFunc(params, t, model);
}

double TrajectoryGeneration::score(double t, HumanoidFixedModel& model, const std::map<std::string, JointModel>& joints,
                                   const Eigen::VectorXd& torques, const Eigen::VectorXd& dq,
                                   const Eigen::VectorXd& ddq, bool isDoubleSupport,
                                   HumanoidFixedModel::SupportFoot supportFoot, std::vector<double>& data) const
{
  return _scoreFunc(t, model, joints, torques, dq, ddq, isDoubleSupport, supportFoot, data);
}
double TrajectoryGeneration::endScore(const Eigen::VectorXd& params, const Trajectories& traj, double score,
                                      std::vector<double>& data, bool verbose) const
{
  return _endScoreFunc(params, traj, score, data, verbose);
}

double TrajectoryGeneration::scoreSim(const Eigen::VectorXd& params, double t, HumanoidSimulation& sim,
                                      std::vector<double>& data) const
{
  return _scoreSimFunc(params, t, sim, data);
}
double TrajectoryGeneration::endScoreSim(const Eigen::VectorXd& params, const Trajectories& traj, double score,
                                         std::vector<double>& data, bool verbose) const
{
  return _endScoreSimFunc(params, traj, score, data, verbose);
}

void TrajectoryGeneration::save(const std::string& filename, const Trajectories& traj, const Eigen::VectorXd& params)
{
  _saveFunc(filename, traj, params);
}

double TrajectoryGeneration::scoreTrajectory(const Eigen::VectorXd& params, bool verbose) const
{
  double cost = checkParameters(params);
  if (cost > 0.0)
  {
    if (verbose)
    {
      std::cout << "Error checkParameters() cost=" << cost << std::endl;
    }
    return cost;
  }
  else
  {
    Trajectories traj = generateTrajectory(params);
    return scoreTrajectory(params, traj, verbose);
  }
}
double TrajectoryGeneration::scoreTrajectory(const Eigen::VectorXd& params, const Trajectories& traj,
                                             bool verbose) const
{
  // Load model parameters
  Eigen::MatrixXd jointData;
  std::map<std::string, size_t> jointName;
  Eigen::MatrixXd inertiaData;
  std::map<std::string, size_t> inertiaName;
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  if (_modelParametersPath != "")
  {
    ReadModelParameters(_modelParametersPath, jointData, jointName, inertiaData, inertiaName, geometryData,
                        geometryName);
  }
  // Joint Model for each DOF
  std::map<std::string, JointModel> joints;
  for (const std::string& name : NamesDOF)
  {
    joints[name] = JointModel();
    if (_modelParametersPath != "" && jointName.count(name) > 0)
    {
      joints[name].setParameters(jointData.row(jointName.at(name)).transpose());
    }
  }
  // Sigmaban fixed model
  Leph::HumanoidFixedModel model(_type, inertiaData, inertiaName, geometryData, geometryName);
  // Retrieve time bounds
  double timeMin = traj.min();
  double timeMax = traj.max();
  double cost = 0.0;
  std::vector<double> data;
  for (double t = timeMin; t <= timeMax; t += 0.01)
  {
    // Compute DOF targets
    Eigen::Vector3d trunkPos;
    Eigen::Vector3d trunkAxis;
    Eigen::Vector3d footPos;
    Eigen::Vector3d footAxis;
    TrajectoriesTrunkFootPos(t, traj, trunkPos, trunkAxis, footPos, footAxis);
    // Check Cartesian State
    double costState = checkState(params, t, trunkPos, trunkAxis, footPos, footAxis);
    if (costState > 0.0)
    {
      if (verbose)
      {
        std::cout << "Error checkState() cost=" << 1000.0 + costState << std::endl;
      }
      cost += 1000.0 + costState;
      continue;
    }
    // Compute kinematics
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
    double boundIKDistance = 0.0;
    bool isIKSuccess = TrajectoriesComputeKinematics(t, traj, model, dq, ddq, &boundIKDistance);
    // Cost near IK bound
    double boundIKThreshold = 1e-2;
    if (boundIKDistance < boundIKThreshold)
    {
      if (verbose)
      {
        std::cout << "Warning boundIKDistance=" << boundIKDistance << std::endl;
      }
      cost += 1000.0 + 1000.0 * (boundIKThreshold - boundIKDistance);
    }
    if (!isIKSuccess)
    {
      if (verbose)
      {
        std::cout << "Error checkIK() cost=" << 1000.0 << std::endl;
      }
      cost += 1000.0;
      continue;
    }
    // Check Joint DOF
    double costDOF = checkDOF(params, t, model.get());
    if (costDOF > 0.0)
    {
      if (verbose)
      {
        std::cout << "Error checkDOF() cost=" << 1000.0 + costDOF << std::endl;
      }
      cost += 1000.0 + costDOF;
      continue;
    }
    // Compute DOF torques
    bool isDoubleSupport;
    HumanoidFixedModel::SupportFoot supportFoot;
    TrajectoriesSupportFootState(t, traj, isDoubleSupport, supportFoot);
    Eigen::VectorXd torques;
    if (isDoubleSupport)
    {
      if (supportFoot == HumanoidFixedModel::LeftSupportFoot)
      {
        torques = model.get().inverseDynamicsClosedLoop("right_foot_tip", nullptr, false, dq, ddq);
      }
      else
      {
        torques = model.get().inverseDynamicsClosedLoop("left_foot_tip", nullptr, false, dq, ddq);
      }
    }
    else
    {
      torques = model.get().inverseDynamics(dq, ddq);
    }
    // Evaluate the trajectory
    cost += score(t, model, joints, torques, dq, ddq, isDoubleSupport, supportFoot, data);
  }
  // Ending trajectory scoring
  cost += endScore(params, traj, cost, data, verbose);

  return cost;
}

double TrajectoryGeneration::scoreSimulation(const Eigen::VectorXd& params, bool verbose) const
{
  double cost = checkParameters(params);
  if (cost > 0.0)
  {
    if (verbose)
    {
      std::cout << "Error checkParameters() cost=" << cost << std::endl;
    }
    return cost;
  }
  else
  {
    Trajectories traj = generateTrajectory(params);
    return scoreSimulation(params, traj, verbose);
  }
}
double TrajectoryGeneration::scoreSimulation(const Eigen::VectorXd& params, const Trajectories& traj,
                                             bool verbose) const
{
  // Load model parameters
  Eigen::MatrixXd jointData;
  std::map<std::string, size_t> jointName;
  Eigen::MatrixXd inertiaData;
  std::map<std::string, size_t> inertiaName;
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  if (_modelParametersPath != "")
  {
    ReadModelParameters(_modelParametersPath, jointData, jointName, inertiaData, inertiaName, geometryData,
                        geometryName);
  }
  // Simulator and goal model initialization
  Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
  Leph::HumanoidSimulation sim(Leph::SigmabanModel, inertiaData, inertiaName, geometryData, geometryName);
  // Assign joint parameters
  for (const std::string& name : Leph::NamesDOF)
  {
    if (jointName.count(name) > 0)
    {
      sim.jointModel(name).setParameters(jointData.row(jointName.at(name)).transpose());
    }
  }
  // Retrieve time bounds
  double timeMin = traj.min();
  double timeMax = traj.max();

  // State initialization
  // Expected initial velocities
  // and accelerations
  Eigen::VectorXd dqInit;
  Eigen::VectorXd ddqInit;
  // Compute DOF target and velocities from Cartesian
  double boundIKDistanceInit = 0.0;
  bool isSuccessInit =
      Leph::TrajectoriesComputeKinematics(timeMin, traj, modelGoal, dqInit, ddqInit, &boundIKDistanceInit);
  // Cost near IK bound
  double boundIKThreshold = 1e-2;
  if (boundIKDistanceInit < boundIKThreshold)
  {
    if (verbose)
    {
      std::cout << "Error init boundIKDistance: t=" << timeMin << " " << boundIKDistanceInit << std::endl;
    }
    return 1000.0 + 1000.0 * (boundIKThreshold - boundIKDistanceInit);
  }
  // IK Error
  if (!isSuccessInit)
  {
    if (verbose)
    {
      std::cout << "Error init IK t=" << timeMin << std::endl;
    }
    return 2000.0;
  }
  // Assign initial DOF state pos and vel
  for (const std::string& name : Leph::NamesDOF)
  {
    sim.setGoal(name, modelGoal.get().getDOF(name));
    sim.setPos(name, modelGoal.get().getDOF(name));
    size_t indexModel = modelGoal.get().getDOFIndex(name);
    sim.setVel(name, dqInit(indexModel));
    // Reset backlash and goal state
    sim.jointModel(name).resetHiddenState();
  }
  for (const std::string& name : Leph::NamesBase)
  {
    sim.setVel(name, 0.0);
  }
  // Put the model flat on left support
  // foot at origin
  sim.putOnGround(Leph::HumanoidFixedModel::LeftSupportFoot);
  sim.putFootAt(0.0, 0.0, Leph::HumanoidFixedModel::LeftSupportFoot);
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
  Eigen::VectorXd targetTrunkVel = modelGoal.get().pointVelocity("trunk", "origin", dqInit);
  // Compute base DOF on sim model
  Eigen::VectorXd baseVel = trunkJac.colPivHouseholderQr().solve(targetTrunkVel);
  // Assign base vel
  sim.setVel("base_roll", baseVel(0));
  sim.setVel("base_pitch", baseVel(1));
  sim.setVel("base_yaw", baseVel(2));
  sim.setVel("base_x", baseVel(3));
  sim.setVel("base_y", baseVel(4));
  sim.setVel("base_z", baseVel(5));

  // Run small time 0.5s for
  // waiting stabilization (backlash)
  // in case of zero initial trunk velocities
  if (baseVel.norm() < 1e-8)
  {
    timeMin -= 0.5;
  }

  // Main simulation loop
  double waitDelay = 3.0;
  double cost = 0.0;
  std::vector<double> data;
  for (double t = timeMin; t <= timeMax + waitDelay; t += 0.01)
  {
    // Compute DOF goal target
    Eigen::Vector3d trunkPos;
    Eigen::Vector3d trunkAxis;
    Eigen::Vector3d footPos;
    Eigen::Vector3d footAxis;
    bool isDoubleSupport;
    Leph::HumanoidFixedModel::SupportFoot supportFoot;
    Leph::TrajectoriesTrunkFootPos(t, traj, trunkPos, trunkAxis, footPos, footAxis);
    Leph::TrajectoriesSupportFootState(t, traj, isDoubleSupport, supportFoot);
    double boundIKDistance = 0.0;
    bool isSuccess = modelGoal.trunkFootIK(supportFoot, trunkPos, Leph::AxisToMatrix(trunkAxis), footPos,
                                           Leph::AxisToMatrix(footAxis), &boundIKDistance);
    // Cost near IK bound
    if (boundIKDistance < boundIKThreshold)
    {
      if (verbose)
      {
        std::cout << "Error IK bound reached: t=" << t << " " << boundIKDistance << std::endl;
      }
      cost += 1000.0 + 1000.0 * (boundIKThreshold - boundIKDistance);
      break;
    }
    // IK Error
    if (!isSuccess)
    {
      if (verbose)
      {
        std::cout << "Error IK t=" << t << std::endl;
      }
      cost += 2000.0;
      break;
    }
    // Assign simulation DOF goal
    for (const std::string& name : Leph::NamesDOF)
    {
      sim.setGoal(name, modelGoal.get().getDOF(name));
    }
    // Run simulation
    try
    {
      for (int k = 0; k < 10; k++)
      {
        sim.update(0.001);
      }
    }
    catch (const std::runtime_error& e)
    {
      if (verbose)
      {
        std::cout << "Error exception: " << e.what() << std::endl;
      }
      return 2000.0;
    }
    // Check Joint DOF
    double costDOF = checkDOF(params, t, sim.model());
    if (costDOF > 0.0)
    {
      if (verbose)
      {
        std::cout << "Error checkDOF() cost=" << 1000.0 + costDOF << std::endl;
      }
      cost += 1000.0 + costDOF;
      break;
    }
    // Detect and penalize falling
    // if trunk orientation is below 45 degrees
    Eigen::Vector3d trunkAngles = sim.model().trunkSelfOrientation();
    if (fabs(trunkAngles.x()) > M_PI / 4.0 || fabs(trunkAngles.y()) > M_PI / 4.0)
    {
      cost += 500.0;
      break;
    }
    // Call user defined score function
    cost += scoreSim(params, t, sim, data);
  }
  // Call user defined end score function
  cost += endScoreSim(params, traj, cost, data, verbose);

  return cost;
}

void TrajectoryGeneration::runOptimization(unsigned int maxIterations, unsigned int restart,
                                           const std::string& filename, unsigned int populationSize, double lambda,
                                           unsigned int elitismLevel, unsigned int verboseIterations,
                                           bool isForwardSimulationOptimization)
{
  // Retrieve initial parameters
  Eigen::VectorXd initParams = initialParameters();
  // Retrieve normalization coefficients
  const Eigen::VectorXd normCoef = normalizationCoefs();
  // Initialization
  _countIteration = 1;
  _bestScore = -1.0;

  // Fitness function
  libcmaes::FitFuncEigen fitness = [this, &normCoef, &isForwardSimulationOptimization](const Eigen::VectorXd& params) {
    if (isForwardSimulationOptimization)
    {
      return this->scoreSimulation(params.array() * normCoef.array());
    }
    else
    {
      return this->scoreTrajectory(params.array() * normCoef.array());
    }
  };
  // Progress function
  libcmaes::ProgressFunc<libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress =
      [this, &filename, &normCoef, &verboseIterations, &isForwardSimulationOptimization](
          const libcmaes::CMAParameters<>& cmaparams, const libcmaes::CMASolutions& cmasols) {
        // Empty case
        if (cmasols.get_best_seen_candidate().get_x_dvec().size() == 0)
        {
          // Call default CMA-ES default progress function
          return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>::_defaultPFunc(cmaparams, cmasols);
        }
        // Retrieve best Trajectories and score
        Eigen::VectorXd params = cmasols.get_best_seen_candidate().get_x_dvec().array() * normCoef.array();
        double score = cmasols.get_best_seen_candidate().get_fvalue();
        if (_bestScore < 0.0 || _bestScore > score)
        {
          _bestParams = params;
          _bestTraj = generateTrajectory(params);
          _bestScore = score;
        }
        // Save current best found
        if (_countIteration % verboseIterations == 0)
        {
          std::cout << "============" << std::endl;
          std::cout << "****** Date: " << currentDate() << std::endl;
          // Save best trajectories and parameters
          this->save(filename, _bestTraj, _bestParams);
          std::cout << "****** Dimension: " << _bestParams.size() << std::endl;
          std::cout << "****** BestScore: " << _bestScore << std::endl;
          std::cout << "****** BestFitness verbose:" << std::endl;
          if (isForwardSimulationOptimization)
          {
            scoreSimulation(_bestParams, true);
          }
          else
          {
            scoreTrajectory(_bestParams, true);
          }
          std::cout << "****** CurrentScore: " << score << std::endl;
          std::cout << "****** CurrentFitness verbose:" << std::endl;
          if (isForwardSimulationOptimization)
          {
            scoreSimulation(params, true);
          }
          else
          {
            scoreTrajectory(params, true);
          }
          std::cout << "============" << std::endl;
        }
        _countIteration++;

        // Call default CMA-ES default progress function
        return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>::_defaultPFunc(cmaparams, cmasols);
      };

  // Show initial info
  double initScore;
  if (isForwardSimulationOptimization)
  {
    initScore = scoreSimulation(initParams, true);
  }
  else
  {
    initScore = scoreTrajectory(initParams, true);
  }
  std::cout << "============" << std::endl;
  std::cout << "Init Score: " << initScore << std::endl;
  std::cout << "Dimension:  " << initParams.size() << std::endl;
  std::cout << "============" << std::endl;

  // CMAES initialization
  libcmaes::CMAParameters<> cmaparams(initParams.array() / normCoef.array(), lambda, populationSize);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(true);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_elitism(elitismLevel);
  cmaparams.set_restarts(restart);
  cmaparams.set_max_iter(maxIterations);

  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams, progress);

  // Retrieve best Trajectories and score
  Eigen::VectorXd params = cmasols.get_best_seen_candidate().get_x_dvec().array() * normCoef.array();
  double score = cmasols.get_best_seen_candidate().get_fvalue();
  _bestParams = params;
  _bestTraj = generateTrajectory(params);
  _bestScore = score;
  std::cout << "############" << std::endl;
  // Save best trajectories and parameters
  save(filename, _bestTraj, _bestParams);
  std::cout << "****** BestScore: " << _bestScore << std::endl;
  std::cout << "****** BestParams: " << _bestParams.transpose() << std::endl;
  std::cout << "############" << std::endl;
}

const Trajectories& TrajectoryGeneration::bestTrajectories() const
{
  return _bestTraj;
}
const Eigen::VectorXd& TrajectoryGeneration::bestParameters() const
{
  return _bestParams;
}
double TrajectoryGeneration::bestScore() const
{
  return _bestScore;
}

}  // namespace Leph
