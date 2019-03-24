#ifndef LEPH_TRAJECTORYGENERATION_HPP
#define LEPH_TRAJECTORYGENERATION_HPP

#include <map>
#include <string>
#include <functional>
#include <Eigen/Dense>
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/JointModel.hpp"
#include "Model/HumanoidSimulation.hpp"

namespace Leph
{
/**
 * TrajectoryGeneration
 *
 * Interface for Smooth Trajectory
 * generation through CMA-ES splines
 * optimization
 */
class TrajectoryGeneration
{
public:
  /**
   * Typedef for user functions defining
   * the optimization goal
   */
  typedef std::function<Trajectories(const Eigen::VectorXd& params)> GenerationFunc;
  typedef std::function<double(const Eigen::VectorXd& params)> CheckParamsFunc;
  typedef std::function<double(const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                               const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                               const Eigen::Vector3d& footAxis)>
      CheckStateFunc;
  typedef std::function<double(const Eigen::VectorXd& params, double t, const HumanoidModel& model)> CheckDOFFunc;
  typedef std::function<double(double t, HumanoidFixedModel& model, const std::map<std::string, JointModel>& joints,
                               const Eigen::VectorXd& torques, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                               bool isDoubleSupport, HumanoidFixedModel::SupportFoot supportFoot,
                               std::vector<double>& data)>
      ScoreFunc;
  typedef std::function<double(const Eigen::VectorXd& params, const Trajectories& traj, double score,
                               std::vector<double>& data, bool verbose)>
      EndScoreFunc;
  typedef std::function<double(const Eigen::VectorXd& params, double t, HumanoidSimulation& sim,
                               std::vector<double>& data)>
      ScoreSimFunc;
  typedef std::function<double(const Eigen::VectorXd& params, const Trajectories& traj, double score,
                               std::vector<double>& data, bool verbose)>
      EndScoreSimFunc;
  typedef std::function<void(const std::string& filename, const Trajectories& traj, const Eigen::VectorXd& params)>
      SaveFunc;

  /**
   * Initialization with
   * humanoid type and an optional
   * filepath to model parameters
   */
  TrajectoryGeneration(RobotType type, const std::string& modelParamsPath = "");

  /**
   * Set the initial parameters for
   * optimization starting point.
   * Defines the parameter dimension.
   */
  void setInitialParameters(const Eigen::VectorXd& params);

  /**
   * Set the normalization coefficients.
   */
  void setNormalizationCoefs(const Eigen::VectorXd& normCoefs);

  /**
   * Set Trajectory Generation function.
   * The function returns the Trajectories spline
   * container from given parameters vector.
   */
  void setTrajectoryGenerationFunc(GenerationFunc func);

  /**
   * Set the Parameters check function.
   * The function returns 0.0 if all given
   * parameters have valid range. It returns
   * positive cost value if given parameters are
   * not in range.
   */
  void setCheckParametersFunc(CheckParamsFunc func);

  /**
   * Set the Cartesian Trunk/Foot state
   * check function.
   * The function returns positive cost value if
   * given time, trunk/foot position and orientation
   * are outside valid bounds.
   */
  void setCheckStateFunc(CheckStateFunc func);

  /**
   * Set the Joint DOF check function.
   * The function returns positive cost value
   * if given time and model DOF state is outside
   * valid bounds.
   */
  void setCheckDOFFunc(CheckDOFFunc func);

  /**
   * Set the scoring function.
   * The function returns positive cost value
   * for the given Humanoid model state, DOF
   * torques, velocities and accelerations.
   */
  void setScoreFunc(ScoreFunc func);

  /**
   * Set the ending scoring trajectory function.
   * The function is called at the end of Trajectory
   * scoring. It returns positive cost value from given
   * Trajectories spline container;
   */
  void setEndScoreFunc(EndScoreFunc func);

  /**
   * Set the scoring function for simulation
   * optimization.
   * The function returns positive cost value
   * for the given Humanoid simulation.
   */
  void setScoreSimFunc(ScoreSimFunc func);

  /**
   * Set the ending scoring trajectory function for
   * simulation optimization.
   * The function is called at the end of Trajectory
   * scoring. It returns positive cost value from given
   * Trajectories spline container;
   */
  void setEndScoreSimFunc(EndScoreSimFunc func);

  /**
   * Set the saving function. The function is called
   * at regular interval (in progress function) and
   * save given current best trajectories
   * and parameters to given filename.
   */
  void setSaveFunc(SaveFunc func);

  /**
   * Return initial parameters
   */
  Eigen::VectorXd initialParameters() const;

  /**
   * Return normalization coefficients
   */
  Eigen::VectorXd normalizationCoefs() const;

  /**
   * Return model parameters path
   */
  const std::string& modelParametersPath() const;

  /**
   * Call Trajectory Generation function
   */
  Trajectories generateTrajectory(const Eigen::VectorXd& params) const;

  /**
   * Call Parameters/State/DOF check function
   */
  double checkParameters(const Eigen::VectorXd& params) const;
  double checkState(const Eigen::VectorXd& params, double t, const Eigen::Vector3d& trunkPos,
                    const Eigen::Vector3d& trunkAxis, const Eigen::Vector3d& footPos,
                    const Eigen::Vector3d& footAxis) const;
  double checkDOF(const Eigen::VectorXd& params, double t, const HumanoidModel& model) const;

  /**
   * Call score function
   */
  double score(double t, HumanoidFixedModel& model, const std::map<std::string, JointModel>& joints,
               const Eigen::VectorXd& torques, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
               bool isDoubleSupport, HumanoidFixedModel::SupportFoot supportFoot, std::vector<double>& data) const;
  double endScore(const Eigen::VectorXd& params, const Trajectories& traj, double score, std::vector<double>& data,
                  bool verbose) const;

  /**
   * Call score function
   * for simulation optimization
   */
  double scoreSim(const Eigen::VectorXd& params, double t, HumanoidSimulation& sim, std::vector<double>& data) const;
  double endScoreSim(const Eigen::VectorXd& params, const Trajectories& traj, double score, std::vector<double>& data,
                     bool verbose) const;

  /**
   * Call saving function
   */
  void save(const std::string& filename, const Trajectories& traj, const Eigen::VectorXd& params);

  /**
   * Build up the Trajectories from
   * given parameters and evaluates it
   * from inverse dynamics.
   */
  double scoreTrajectory(const Eigen::VectorXd& params, bool verbose = false) const;
  double scoreTrajectory(const Eigen::VectorXd& params, const Trajectories& traj, bool verbose = false) const;

  /**
   * Build up the Trajectories fom
   * given parameters and evatuates it
   * from forward dynamics simulation.
   */
  double scoreSimulation(const Eigen::VectorXd& params, bool verbose = false) const;
  double scoreSimulation(const Eigen::VectorXd& params, const Trajectories& traj, bool verbose = false) const;

  /**
   * Run the CMA-ES Trajectories optimization
   * with given algorithm configuration.
   */
  void runOptimization(unsigned int maxIterations, unsigned int restart, const std::string& filename = "",
                       unsigned int populationSize = 10, double lambda = -1.0, unsigned int elitismLevel = 1,
                       unsigned int verboseIterations = 100, bool isForwardSimulationOptimization = false);

  /**
   * Access to best found Trajectories,
   * score and parameters
   */
  const Trajectories& bestTrajectories() const;
  const Eigen::VectorXd& bestParameters() const;
  double bestScore() const;

private:
  /**
   * Humanoid robot type
   */
  RobotType _type;

  /**
   * If not empty, a filepath to
   * joint, inertia and geometry model
   * parameters to be loaded and used.
   */
  std::string _modelParametersPath;

  /**
   * Initial parameters for
   * optimization process
   */
  Eigen::VectorXd _initialParameters;

  /**
   * Normalization coefficients
   */
  Eigen::VectorXd _normCoefs;

  /**
   * User functions
   */
  GenerationFunc _generateFunc;
  CheckParamsFunc _checkParamsFunc;
  CheckStateFunc _checkStateFunc;
  CheckDOFFunc _checkDOFFunc;
  ScoreFunc _scoreFunc;
  EndScoreFunc _endScoreFunc;
  ScoreSimFunc _scoreSimFunc;
  EndScoreSimFunc _endScoreSimFunc;
  SaveFunc _saveFunc;

  /**
   * Best found trajectories and
   * associated score and parameters
   */
  Trajectories _bestTraj;
  Eigen::VectorXd _bestParams;
  double _bestScore;

  /**
   * Counter for CMAES progress function
   */
  long _countIteration;
};

}  // namespace Leph

#endif
