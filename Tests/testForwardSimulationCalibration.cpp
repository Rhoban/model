#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Model/JointModel.hpp"
#include "Model/RBDLRootUpdate.h"
#include "Types/MapSeries.hpp"
#include "Utils/Angle.h"
#include "Model/NamesModel.h"

#ifdef LEPH_VIEWER_ENABLED
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"
#endif

/**
 * Names of frame used for fitness error evaluation
 */
static std::vector<std::string> namesFrameFitness = {
  /*
  "trunk",
  "right_foot_tip",
  "camera",
  */
};
static std::vector<std::string> namesDOFFitness = {
  "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
  "left_hip_yaw",  "left_hip_pitch",  "left_hip_roll",  "left_knee",  "left_ankle_pitch",  "left_ankle_roll",
};

/**
 * Names of frame whose inertia data is optimized
 */
static std::vector<std::string> namesFrameInertia = {
  "trunk",           "right_hip_yaw",     "right_hip_pitch",  "right_hip_roll",
  "right_knee",      "right_ankle_pitch", "right_ankle_roll", "left_hip_yaw",
  "left_hip_pitch",  "left_hip_roll",     "left_knee",        "left_ankle_pitch",
  "left_ankle_roll",
};

/**
 * Names of DOF whose Joint Model is uniquely optimized
 */
static std::vector<std::string> namesDOFJoint = {
  "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
  "left_hip_yaw",  "left_hip_pitch",  "left_hip_roll",  "left_knee",  "left_ankle_pitch",  "left_ankle_roll",
};

/**
 * Global inertia default data
 * and name
 */
static Eigen::MatrixXd defaultInertiaData;
static std::map<std::string, size_t> defaultInertiaName;

/**
 * Global joint parameters size
 */
static size_t sizeJointParameters;
static size_t sizeJointAllParameters;

/**
 * Global configuration is inertia optimized
 */
static bool isOptimizationInertia = false;

/**
 * Global configuration is single Joint Model optimized
 */
static bool isOptimizationJoint = false;

/**
 * Global configuration is parameters
 * bound to positive
 */
static bool isParametersBoundPositive = true;

/**
 * Global CMA-ES configuration
 */
static unsigned int cmaesMaxIterations = 10000;
static unsigned int cmaesRestarts = 5;
static unsigned int cmaesLambda = 10;
static double cmaesSigma = -1.0;

/**
 * Global configuration is parameters
 * normalization is disable
 */
static bool isParametersNormalization = true;

/**
 * Load and assign default inertia form model
 */
static void loadDefaultInertiaData()
{
  Leph::HumanoidModel model(Leph::SigmabanModel, "left_foot_tip", false);
  defaultInertiaData = model.getInertiaData();
  defaultInertiaName = model.getInertiaName();
}

static double scoreFitness(const std::string& filename, const Eigen::VectorXd& parameters, int verbose,
                           double* sumError = nullptr, double* countError = nullptr, double* maxError = nullptr,
                           Eigen::VectorXd* maxAllError = nullptr)
{
  // Load data into MapSeries
  Leph::MapSeries logs;
  logs.importData(filename);
  // Print statistics
  if (verbose >= 1)
  {
    std::cout << "Loaded " << filename << ": " << logs.dimension() << " series from " << logs.timeMin() << "s to "
              << logs.timeMax() << "s with length " << logs.timeMax() - logs.timeMin() << "s" << std::endl;
  }
  double min = logs.timeMin();
  double max = logs.timeMax();

  // Assign inertia data
  Eigen::MatrixXd currentInertiaData = defaultInertiaData;
  if (isOptimizationInertia)
  {
    size_t index = 0;
    for (const std::string& name : namesFrameInertia)
    {
      currentInertiaData.row(defaultInertiaName.at(name)) =
          parameters.segment(sizeJointAllParameters + index * 10, 10).transpose();
      index++;
    }
    // Check positive inertia parameters
    double tmpCost = 0.0;
    for (size_t i = 0; i < (size_t)currentInertiaData.rows(); i++)
    {
      // 0 Mass
      if (currentInertiaData(i, 0) <= 0.0)
      {
        tmpCost += 1000.0 - 1000.0 * currentInertiaData(i, 0);
      }
      // 4 Ixx
      if (currentInertiaData(i, 4) <= 0.0)
      {
        tmpCost += 1000.0 - 1000.0 * currentInertiaData(i, 4);
      }
      // 7 Iyy
      if (currentInertiaData(i, 7) <= 0.0)
      {
        tmpCost += 1000.0 - 1000.0 * currentInertiaData(i, 7);
      }
      // 9 Izz
      if (currentInertiaData(i, 9) <= 0.0)
      {
        tmpCost += 1000.0 - 1000.0 * currentInertiaData(i, 9);
      }
    }
    if (tmpCost > 0.0)
    {
      return tmpCost;
    }
  }

  // Full humanoid model
  Leph::HumanoidModel modelSim(Leph::SigmabanModel, "left_foot_tip", false, currentInertiaData, defaultInertiaName);
  Leph::HumanoidModel modelRead(Leph::SigmabanModel, "left_foot_tip", false);

  // Simulator
  Leph::ForwardSimulation sim(modelSim);
  sim.setJointModelParameters(parameters.segment(0, sizeJointParameters));
  // Initialization
  for (const std::string& name : Leph::NamesDOF)
  {
    sim.positions()(modelSim.getDOFIndex(name)) = logs.get("read:" + name, min);
    sim.goals()(modelSim.getDOFIndex(name)) = logs.get("read:" + name, min);
    sim.velocities()(modelSim.getDOFIndex(name)) = 0.0;
  }
  if (isOptimizationJoint)
  {
    size_t index = 0;
    for (const std::string& name : namesDOFJoint)
    {
      sim.jointModel(name).setParameters(
          parameters.segment(sizeJointParameters + index * sizeJointParameters, sizeJointParameters));
      index++;
    }
  }

  // Right foot contact
  Leph::RBDL::ConstraintSet constraints;
  constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
  /*
  //Z 0
  constraints.AddConstraint(
      modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_1")),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  //X 1
  constraints.AddConstraint(
      modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_1")),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
      Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  //Y 2
  constraints.AddConstraint(
      modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_1")),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
      Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));
  //Z 3
  constraints.AddConstraint(
      modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_2")),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  //X 4
  constraints.AddConstraint(
      modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_2")),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
      Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  //Z 5
  constraints.AddConstraint(
      modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_3")),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
      Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  */
  constraints.Bind(modelSim.getRBDLModel());
  sim.computeImpulses(constraints);

  // Main loop
  double tmpMax = -1.0;
  double tmpCount = 0.0;
  double tmpSum = 0.0;
  double tmpMaxTime = 0.0;
  Eigen::VectorXd tmpMaxAll(namesFrameFitness.size() + namesDOFJoint.size());
  for (size_t i = 0; i < namesFrameFitness.size() + namesDOFFitness.size(); i++)
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
  for (double t = min; t < max; t += incrStep)
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
    for (const std::string& name : Leph::NamesDOF)
    {
      sim.goals()(modelSim.getDOFIndex(name)) = logs.get("goal:" + name, t);
      modelRead.setDOF(name, logs.get("read:" + name, t));
    }
    for (int k = 0; k < incrLoop; k++)
    {
      sim.update(0.001);
      // sim.update(0.001, &constraints);
    }
    Leph::VectorLabel vect;
    size_t index = 0;
    for (const std::string& name : namesFrameFitness)
    {
      Eigen::Vector3d posCartSim = modelSim.position(name, "left_foot_tip");
      Eigen::Vector3d posCartRead = modelRead.position(name, "left_foot_tip");
      double error = (posCartSim - posCartRead).squaredNorm();
      tmpSum += error;
      tmpCount += 1.0;
      if (tmpMax < 0.0 || tmpMax < error)
      {
        tmpMax = error;
        tmpMaxTime = t;
        tmpMaxName = name;
      }
      if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < error)
      {
        tmpMaxAll(index) = error;
      }
      index++;
    }
    for (const std::string& name : namesDOFFitness)
    {
      double error = pow(180.0 / M_PI * Leph::AngleDistance(modelSim.getDOF(name), modelRead.getDOF(name)), 2);
      tmpSum += error;
      tmpCount += 1.0;
      if (tmpMax < 0.0 || tmpMax < error)
      {
        tmpMax = error;
        tmpMaxTime = t;
        tmpMaxName = name;
      }
      if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < error)
      {
        tmpMaxAll(index) = error;
      }
      index++;
    }
#ifdef LEPH_VIEWER_ENABLED
    if (verbose >= 2)
    {
      for (const std::string& name : Leph::NamesDOF)
      {
        vect.setOrAppend("t", t);
        vect.setOrAppend("read:" + name, 180.0 / M_PI * logs.get("read:" + name, t));
        vect.setOrAppend("goal:" + name, 180.0 / M_PI * logs.get("goal:" + name, t));
        vect.setOrAppend("sim:" + name, 180.0 / M_PI * sim.positions()(modelSim.getDOFIndex(name)));
      }
      plot.add(vect);
    }
    if (verbose >= 3)
    {
      Leph::ModelDraw(modelSim, *viewer, 1.0);
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
    for (size_t i = 0; i < namesFrameFitness.size() + namesDOFFitness.size(); i++)
    {
      if (tmpMaxAll(i) > maxAllError->operator()(i))
      {
        maxAllError->operator()(i) = tmpMaxAll(i);
      }
    }
  }

  return 0.0;
}

int main()
{
  // Load default inertia data from model
  loadDefaultInertiaData();

  // Load data into MapSeries
  std::vector<std::string> filenames = {
    "../../These/Data/logs-2016-11-29_model_calibration/calibration_log_foot_x_2016-11-29-19-45-36.mapseries",
    "../../These/Data/logs-2016-11-29_model_calibration/calibration_log_trajectory_2016-11-29-20-06-50.mapseries",
    "../../These/Data/logs-2016-11-29_model_calibration/calibration_log_trajectory_2016-11-29-20-08-47.mapseries",
    "../../These/Data/logs-2016-11-29_model_calibration/calibration_log_trajectory_2016-11-29-20-10-44.mapseries",
    //"../../These/Data/logs-2016-11-29_model_calibration/calibration_log_trajectory_2016-11-29-20-12-32.mapseries",

    //"../../These/Data/logs-2016-10-30_model_calibration/backlash/calibration_log_foot_x_2016-11-02-11-37-10.mapseries",
    //"../../These/Data/logs-2016-10-30_model_calibration/backlash/calibration_log_foot_x_2016-11-02-11-37-35.mapseries",
    //"../../These/Data/logs-2016-10-30_model_calibration/backlash/calibration_log_foot_x_2016-11-02-11-37-21.mapseries",
    //"../../These/Data/logs-2016-10-30_model_calibration/backlash/calibration_log_foot_x_2016-11-02-11-37-50.mapseries",
    //"../../These/Data/logs-2016-10-30_model_calibration/kick/calibration_log_trajectory_2016-11-01-10-45-08.mapseries",
    /*
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-07-37.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-08-03.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-08-19.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-08-40.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-09-06.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_y_2016-10-30-17-10-58.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_y_2016-10-30-17-11-12.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-03-37.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-03-56.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-04-14.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-04-33.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-04-56.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-07-02.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_x_2016-10-30-17-09-24.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_x_2016-10-30-17-09-44.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_x_2016-10-30-17-10-06.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_y_2016-10-30-17-10-19.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_y_2016-10-30-17-10-31.mapseries",
    "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_y_2016-10-30-17-10-45.mapseries",
    */
  };

  // Initialize joint model parameters
  Leph::JointModel tmpJoint;
  Eigen::VectorXd initParams = tmpJoint.getParameters();
  sizeJointParameters = initParams.size();
  sizeJointAllParameters = sizeJointParameters;

  if (isOptimizationJoint)
  {
    // Assign initial joint model parameters
    size_t index = 0;
    sizeJointAllParameters = (1 + namesDOFJoint.size()) * sizeJointParameters;
    initParams.conservativeResize(sizeJointAllParameters);
    for (const std::string& name : namesDOFJoint)
    {
      initParams.segment(sizeJointParameters + index * sizeJointParameters, sizeJointParameters) =
          initParams.segment(0, sizeJointParameters);
      index++;
    }
  }
  if (isOptimizationInertia)
  {
    // Assign initial inertia parameters
    size_t index = 0;
    initParams.conservativeResize(sizeJointAllParameters + namesFrameInertia.size() * 10);
    for (const std::string& name : namesFrameInertia)
    {
      initParams.segment(sizeJointAllParameters + index * 10, 10) =
          defaultInertiaData.row(defaultInertiaName.at(name)).transpose();
      index++;
    }
  }

  // Normalization coefficient
  Eigen::VectorXd coef = initParams;
  Eigen::VectorXd coefInv = initParams;
  for (size_t i = 0; i < (size_t)coef.size(); i++)
  {
    if (isParametersNormalization)
    {
      if (fabs(coef(i)) < 1e-4)
      {
        coef(i) = 1.0;
      }
      coefInv(i) = 1.0 / coef(i);
    }
    else
    {
      coef(i) = 1.0;
      coefInv(i) = 1.0;
    }
  }

  // Initial verbose
  for (size_t i = 0; i < filenames.size(); i++)
  {
    scoreFitness(filenames[i], initParams, 3);
  }

  // Normalization of parameters
  initParams = coefInv.array() * initParams.array();
  Eigen::VectorXd bestParams = initParams;
  double bestScore = -1.0;
  int iteration = 1;

  // Fitness function
  libcmaes::FitFuncEigen fitness = [&filenames, &coef](const Eigen::VectorXd& params) {
    // Check positive parameters
    double cost = 0.0;
    if (isParametersBoundPositive)
    {
      bool isValide = true;
      for (size_t i = 0; i < (size_t)params.size(); i++)
      {
        if (i < sizeJointAllParameters && params(i) < 0.0)
        {
          cost += 1000.0 - 1000.0 * params(i);
          isValide = false;
        }
      }
      if (!isValide)
      {
        return cost;
      }
    }
    // Iterate over on all logs
    double sumError = 0.0;
    double countError = 0.0;
    double maxError = -1.0;
    Eigen::VectorXd maxAllError(namesFrameFitness.size() + namesDOFFitness.size());
    for (size_t i = 0; i < namesFrameFitness.size() + namesDOFFitness.size(); i++)
    {
      maxAllError(i) = -1.0;
    }
    for (size_t i = 0; i < filenames.size(); i++)
    {
      try
      {
        cost += scoreFitness(filenames[i], coef.array() * params.array(), 0, &sumError, &countError, &maxError,
                             &maxAllError);
      }
      catch (const std::runtime_error& e)
      {
        cost += 10000.0;
      }
    }
    if (countError > 0.0 && maxError > 0.0)
    {
      return cost + 0.6 * (sumError / countError) + 0.2 * (maxAllError.mean()) + 0.2 * (maxError);
    }
    else
    {
      return cost;
    }
  };

  // Progress function
  libcmaes::ProgressFunc<libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress =
      [&bestParams, &bestScore, &iteration, &coef, &filenames](const libcmaes::CMAParameters<>& cmaparams,
                                                               const libcmaes::CMASolutions& cmasols) {
        // Retrieve best Trajectories and score
        Eigen::VectorXd params = cmasols.get_best_seen_candidate().get_x_dvec();
        double score = cmasols.get_best_seen_candidate().get_fvalue();
        if (!std::isnan(score) && (bestScore < 0.0 || bestScore > score))
        {
          bestParams = params;
          bestScore = score;
        }
        if (iteration % 50 == 0)
        {
          std::cout << "============" << std::endl;
          std::cout << "Dimension: " << params.size() << std::endl;
          std::cout << "BestScore: " << bestScore << std::endl;
          std::cout << "BestParams: ";
          for (size_t i = 0; i < (size_t)bestParams.size(); i++)
          {
            std::cout << std::setprecision(10) << bestParams(i) * coef(i);
            if (i != (size_t)bestParams.size() - 1)
            {
              std::cout << ", ";
            }
            else
            {
              std::cout << ";" << std::endl;
            }
          }
          std::cout << "Score: " << score << std::endl;
          std::cout << "Params: " << (params.array() * coef.array()).transpose() << std::endl;
          std::cout << "============" << std::endl;
          for (size_t i = 0; i < filenames.size(); i++)
          {
            scoreFitness(filenames[i], coef.array() * bestParams.array(), 1);
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
  cmaparams.set_elitism(true);
  cmaparams.set_restarts(cmaesRestarts);
  cmaparams.set_max_iter(cmaesMaxIterations);
  cmaparams.set_ftolerance(1e-9);

  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams, progress);

  // Retrieve best Trajectories and score
  bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
  bestScore = cmasols.get_best_seen_candidate().get_fvalue();

  // Final verbose
  for (size_t i = 0; i < filenames.size(); i++)
  {
    scoreFitness(filenames[i], coef.array() * bestParams.array(), 3);
  }
  std::cout << "BestParams: " << (coef.array() * bestParams.array()).transpose() << std::endl;
  std::cout << "BestScore: " << bestScore << std::endl;

  return 0;
}
