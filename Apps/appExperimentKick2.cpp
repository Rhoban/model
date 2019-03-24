#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include <lwpr_eigen.hpp>
#include <gp.h>
#include <rprop.h>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Spline/CubicSpline.hpp"
#include "Spline/SplineContainer.hpp"
#include "Spline/FittedSpline.hpp"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"
#include "Utils/LWPRUtils.h"
#include "LinearRegression/SimpleLinearRegression.hpp"
#include "Utils/ComputeModelData.h"
#include "Model/NamesModel.h"

/**
 * Learning task summary
 */
struct LearningTask
{
  std::vector<std::string> logfiles;
  std::vector<std::string> inputs;
  std::vector<size_t> inputsLag;
  std::vector<size_t> inputsDelta;
  std::vector<std::string> outputs;
  std::function<void(Leph::MatrixLabel& logs)> funcComputeModelData;
};

/**
 * Learning data used to proccess a learning task
 */
struct LearningData
{
  std::vector<Leph::MatrixLabel> logs;
  std::vector<Eigen::VectorXd> inputsLearnData;
  std::vector<Eigen::VectorXd> outputsLearnData;
  std::vector<Eigen::VectorXd> inputsTestData;
  std::vector<Eigen::VectorXd> outputsTestData;
};

/**
 * Initialize and return models
 */
LWPR_Object learningInitModelLWPR(const LearningTask& task)
{
  LWPR_Object model(2 * task.inputs.size(), task.outputs.size());
  // Normalisation value for each input dimension (>0)
  Eigen::VectorXd vectNormIn(2 * task.inputs.size());
  /* TODO
  vectNormIn <<
      0.01, 0.01,
      0.01, 0.01,
      0.01 ,0.01,
      0.1, 0.1,
      0.1, 0.1,
      0.1 ,0.1,
      0.01, 0.01,
      0.01, 0.01,
      0.01 ,0.01;
  */
  /*
  vectNormIn <<
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01,
      0.1, 0.1,
      0.1, 0.1,
      0.1, 0.1,
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01,
      0.1, 0.1,
      0.1, 0.1,
      0.1, 0.1,
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01,
      0.01, 0.01;
  */
  // model.normIn(vectNormIn);
  model.normIn(0.01);
  // Use only diagonal matrix. Big speed up in high dimension
  // but lower performance in complex learning.
  model.diagOnly(false);
  // Automatic tunning of distance metric
  model.useMeta(false);
  model.updateD(false);
  // Larger value enforce wider receptive field (>0)
  model.penalty(1e-6);
  // Set diagonal (input_dim) or complet (input_dim*input_dim)
  // initial distance matrix (>0)
  model.setInitD(0.1);
  return model;
}
std::vector<libgp::GaussianProcess> learningInitModelGP(const LearningTask& task)
{
  std::vector<libgp::GaussianProcess> models;
  for (size_t i = 0; i < task.outputs.size(); i++)
  {
    models.push_back(libgp::GaussianProcess(2 * task.inputs.size(), "CovSum ( CovSEiso, CovNoise)"));
  }
  return models;
}

/**
 * Build inputs and outputs Vector at given logs index
 * Return empty vector if given index is not usable data
 */
Eigen::VectorXd learningRetrieveInputs(const LearningTask& task, const Leph::MatrixLabel& logs, size_t index)
{
  Eigen::VectorXd in(2 * task.inputs.size());
  for (size_t i = 0; i < task.inputs.size(); i++)
  {
    size_t lag = task.inputsLag[i];
    size_t delta = task.inputsDelta[i];
    if (index < lag || index < lag + delta || !logs[index - lag].exist(task.inputs[i]) ||
        !logs[index - lag - delta].exist(task.inputs[i]))
    {
      return Eigen::VectorXd();
    }
    in(2 * i) = logs[index - lag](task.inputs[i]);
    in(2 * i + 1) = logs[index - lag - delta](task.inputs[i]);
  }
  return in;
}
Eigen::VectorXd learningRetrieveOutputs(const LearningTask& task, const Leph::MatrixLabel& logs, size_t index)
{
  Eigen::VectorXd out(task.outputs.size());
  for (size_t i = 0; i < task.outputs.size(); i++)
  {
    if (!logs[index].exist(task.outputs[i]))
    {
      return Eigen::VectorXd();
    }
    out(i) = logs[index](task.outputs[i]);
  }
  return out;
}

/**
 * Init and return Learning Data
 */
LearningData learningInit(const LearningTask& task)
{
  LearningData data;
  // Load data from file and compute model
  for (size_t i = 0; i < task.logfiles.size(); i++)
  {
    data.logs.push_back(Leph::MatrixLabel());
    data.logs[i].load(task.logfiles[i]);
    task.funcComputeModelData(data.logs[i]);
  }
  // Compute learn and test data points
  size_t size = data.logs.size();
  if (size > 1)
  {
    // Split logs in two
    // Half rounded to upper integer
    size_t middle = (size + 1) / 2 - 1;
    // Add learn data
    for (size_t k = 0; k <= middle; k++)
    {
      for (size_t i = 0; i < data.logs[k].size(); i++)
      {
        Eigen::VectorXd in = learningRetrieveInputs(task, data.logs[k], i);
        Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs[k], i);
        if (in.size() > 0 && out.size() > 0)
        {
          data.inputsLearnData.push_back(in);
          data.outputsLearnData.push_back(out);
        }
      }
    }
    // Add test data
    for (size_t k = middle + 1; k < size; k++)
    {
      for (size_t i = 0; i < data.logs[k].size(); i++)
      {
        Eigen::VectorXd in = learningRetrieveInputs(task, data.logs[k], i);
        Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs[k], i);
        if (in.size() > 0 && out.size() > 0)
        {
          data.inputsTestData.push_back(in);
          data.outputsTestData.push_back(out);
        }
      }
    }
  }
  else if (data.logs.size() == 1)
  {
    // Split the log in two sequence
    // Half rounded to upper integer
    size_t middle = (data.logs.front().size() + 1) / 2 - 1;
    // Add learn data
    for (size_t i = 0; i <= middle; i++)
    {
      Eigen::VectorXd in = learningRetrieveInputs(task, data.logs.front(), i);
      Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs.front(), i);
      if (in.size() > 0 && out.size() > 0)
      {
        data.inputsLearnData.push_back(in);
        data.outputsLearnData.push_back(out);
      }
    }
    // Add test data
    for (size_t i = middle + 1; i < data.logs.front().size(); i++)
    {
      Eigen::VectorXd in = learningRetrieveInputs(task, data.logs.front(), i);
      Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs.front(), i);
      if (in.size() > 0 && out.size() > 0)
      {
        data.inputsTestData.push_back(in);
        data.outputsTestData.push_back(out);
      }
    }
  }

  return data;
}

void learningDoLearn(const LearningData& data, LWPR_Object& model, bool completeData = false)
{
  for (size_t i = 0; i < data.inputsLearnData.size(); i++)
  {
    model.update(data.inputsLearnData[i], data.outputsLearnData[i]);
  }
  if (!completeData)
  {
    model.writeBinary("/tmp/model.lwpr");
    return;
  }
  for (size_t i = 0; i < data.inputsTestData.size(); i++)
  {
    model.update(data.inputsTestData[i], data.outputsTestData[i]);
  }
  model.writeBinary("/tmp/model.lwpr");
}
void learningDoLearn(const LearningData& data, std::vector<libgp::GaussianProcess>& models)
{
  for (size_t k = 0; k < models.size(); k++)
  {
    for (size_t i = 0; i < data.inputsLearnData.size(); i++)
    {
      models[k].add_pattern(data.inputsLearnData[i], data.outputsLearnData[i](k));
    }
    libgp::RProp rprop;
    rprop.init();
    std::string modelName = "/tmp/model_" + std::to_string(k) + ".gp";
    rprop.maximize(&models[k], 50, true);
    models[k].write(modelName.c_str());
    std::cout << "ModelGP " << k << ": " << models[k].covf().get_loghyper().transpose() << std::endl;
  }
  /*
  Eigen::VectorXd hyperParams1(3);
  hyperParams1 << -1.8333,  2.89507, -4.66417;
  Eigen::VectorXd hyperParams2(3);
  hyperParams2 << -1.78393,  3.01733, -4.78753;
  models[0].covf().set_loghyper(hyperParams1);
  models[1].covf().set_loghyper(hyperParams2);
  */
}

double learningDoTesting(const LearningData& data, LWPR_Object& model)
{
  double sumSquareError = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < data.inputsTestData.size(); i++)
  {
    Eigen::VectorXd yp = model.predict(data.inputsTestData[i], 0.0);
    sumSquareError += (yp - data.outputsTestData[i]).squaredNorm();
    count++;
  }

  return sqrt(sumSquareError / count);
}
double learningDoTesting(const LearningData& data, std::vector<libgp::GaussianProcess>& models)
{
  double sumSquareError = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < data.inputsTestData.size(); i++)
  {
    for (size_t k = 0; k < models.size(); k++)
    {
      double yp = models[k].f(data.inputsTestData[i]);
      sumSquareError += pow(yp - data.outputsTestData[i](k), 2);
    }
    count++;
  }

  return sqrt(sumSquareError / count);
}

void learningTestStats(const LearningData& data, LWPR_Object& model)
{
  if (data.inputsTestData.size() == 0 || data.outputsTestData.size() == 0)
  {
    return;
  }
  size_t sizeIn = data.inputsTestData.front().size();
  size_t sizeOut = data.outputsTestData.front().size();
  Eigen::VectorXd sumSquareError = Eigen::VectorXd::Zero(sizeOut);
  Eigen::VectorXd maxError = Eigen::VectorXd::Constant(sizeOut, -1.0);
  Eigen::VectorXd sumMaxW = Eigen::VectorXd::Zero(sizeOut);
  Eigen::VectorXd minMaxW = Eigen::VectorXd::Constant(sizeOut, -1.0);
  Eigen::VectorXd sumInputs = Eigen::VectorXd::Zero(sizeIn);
  Eigen::VectorXd sumOutputs = Eigen::VectorXd::Zero(sizeOut);
  Eigen::VectorXd sumSquaredInputs = Eigen::VectorXd::Zero(sizeIn);
  Eigen::VectorXd sumSquaredOutputs = Eigen::VectorXd::Zero(sizeOut);
  size_t count = 0;
  for (size_t i = 0; i < data.inputsTestData.size(); i++)
  {
    Eigen::VectorXd confidence;
    Eigen::VectorXd maxW;
    Eigen::VectorXd yp = model.predict(data.inputsTestData[i], confidence, maxW, 0.0);
    Eigen::VectorXd error = yp - data.outputsTestData[i];
    sumSquareError += error.cwiseAbs2();
    for (size_t j = 0; j < sizeOut; j++)
    {
      if (maxError(j) < 0.0 || maxError(j) < fabs(error(j)))
      {
        maxError(j) = fabs(error(j));
      }
      if (minMaxW(j) < 0.0 || minMaxW(j) > maxW(j))
      {
        minMaxW(j) = maxW(j);
      }
    }
    sumMaxW += maxW;
    sumInputs += data.inputsTestData[i];
    sumOutputs += data.outputsTestData[i];
    sumSquaredInputs += data.inputsTestData[i].cwiseAbs2();
    sumSquaredOutputs += data.outputsTestData[i].cwiseAbs2();
    count++;
  }
  if (count == 0)
  {
    std::cout << "No Statistic" << std::endl;
    return;
  }
  sumSquareError /= count;
  sumMaxW /= count;
  std::cout << "Testing Stats:" << std::endl;
  std::cout << "Count=" << count << std::endl;
  std::cout << "MSE: " << sumSquareError.transpose() << std::endl;
  std::cout << "RMSE: " << sumSquareError.cwiseSqrt().transpose() << std::endl;
  std::cout << "MaxError: " << maxError.transpose() << std::endl;
  std::cout << "MeanMaxW: " << sumMaxW.transpose() << std::endl;
  std::cout << "MinMaxW: " << minMaxW.transpose() << std::endl;
  sumInputs /= count;
  sumOutputs /= count;
  std::cout << "Mean inputs: " << sumInputs.transpose() << std::endl;
  std::cout << "Mean outputs: " << sumOutputs.transpose() << std::endl;
  sumSquaredInputs /= count;
  sumSquaredOutputs /= count;
  sumSquaredInputs -= sumInputs.cwiseAbs2();
  sumSquaredOutputs -= sumOutputs.cwiseAbs2();
  std::cout << "StdDev inputs: " << sumSquaredInputs.cwiseSqrt().transpose() << std::endl;
  std::cout << "StdDev outputs: " << sumSquaredOutputs.cwiseSqrt().transpose() << std::endl;
  std::cout << "Var inputs: " << sumSquaredInputs.transpose() << std::endl;
  std::cout << "Var outputs: " << sumSquaredOutputs.transpose() << std::endl;
  Leph::LWPRPrint(model);
}
void learningTestStats(const LearningData& data, std::vector<libgp::GaussianProcess>& models)
{
  if (data.inputsTestData.size() == 0 || data.outputsTestData.size() == 0)
  {
    return;
  }
  size_t sizeIn = data.inputsTestData.front().size();
  size_t sizeOut = data.outputsTestData.front().size();
  Eigen::VectorXd sumSquareError = Eigen::VectorXd::Zero(sizeOut);
  Eigen::VectorXd maxError = Eigen::VectorXd::Constant(sizeOut, -1.0);
  Eigen::VectorXd sumInputs = Eigen::VectorXd::Zero(sizeIn);
  Eigen::VectorXd sumOutputs = Eigen::VectorXd::Zero(sizeOut);
  Eigen::VectorXd sumSquaredInputs = Eigen::VectorXd::Zero(sizeIn);
  Eigen::VectorXd sumSquaredOutputs = Eigen::VectorXd::Zero(sizeOut);
  size_t count = 0;
  for (size_t i = 0; i < data.inputsTestData.size(); i++)
  {
    for (size_t k = 0; k < models.size(); k++)
    {
      double yp = models[k].f(data.inputsTestData[i]);
      double error = yp - data.outputsTestData[i](k);
      sumSquareError(k) += error * error;
      for (size_t j = 0; j < sizeOut; j++)
      {
        if (maxError(j) < 0.0 || maxError(j) < fabs(error))
        {
          maxError(j) = fabs(error);
        }
      }
    }
    sumInputs += data.inputsTestData[i];
    sumOutputs += data.outputsTestData[i];
    sumSquaredInputs += data.inputsTestData[i].cwiseAbs2();
    sumSquaredOutputs += data.outputsTestData[i].cwiseAbs2();
    count++;
  }
  if (count == 0)
  {
    std::cout << "No Statistic" << std::endl;
    return;
  }
  sumSquareError /= count;
  std::cout << "Testing Stats:" << std::endl;
  std::cout << "Count=" << count << std::endl;
  std::cout << "MSE: " << sumSquareError.transpose() << std::endl;
  std::cout << "RMSE: " << sumSquareError.cwiseSqrt().transpose() << std::endl;
  std::cout << "MaxError: " << maxError.transpose() << std::endl;
  sumInputs /= count;
  sumOutputs /= count;
  std::cout << "Mean inputs: " << sumInputs.transpose() << std::endl;
  std::cout << "Mean outputs: " << sumOutputs.transpose() << std::endl;
  sumSquaredInputs /= count;
  sumSquaredOutputs /= count;
  sumSquaredInputs -= sumInputs.cwiseAbs2();
  sumSquaredOutputs -= sumOutputs.cwiseAbs2();
  std::cout << "StdDev inputs: " << sumSquaredInputs.cwiseSqrt().transpose() << std::endl;
  std::cout << "StdDev outputs: " << sumSquaredOutputs.cwiseSqrt().transpose() << std::endl;
  std::cout << "Var inputs: " << sumSquaredInputs.transpose() << std::endl;
  std::cout << "Var outputs: " << sumSquaredOutputs.transpose() << std::endl;
}

void learningOptimize(LearningTask& task, size_t indexBegin, size_t indexEnd)
{
  {
    size_t bestLag = 0;
    double bestScore = -1.0;
    for (size_t k = 0; k < 20; k++)
    {
      for (size_t j = indexBegin; j <= indexEnd; j++)
      {
        task.inputsLag[j] = k;
      }
      LWPR_Object model = learningInitModelLWPR(task);
      LearningData data = learningInit(task);
      learningDoLearn(data, model);
      double score = learningDoTesting(data, model);
      if (bestScore < 0.0 || bestScore > score)
      {
        bestScore = score;
        bestLag = k;
      }
      std::cout << "Optimizing lag=" << k << " score=" << score << std::endl;
    }
    for (size_t j = indexBegin; j <= indexEnd; j++)
    {
      task.inputsLag[j] = bestLag;
    }
  }
  {
    size_t bestDelta = 0;
    double bestScore = -1.0;
    for (size_t k = 0; k < 20; k++)
    {
      for (size_t j = indexBegin; j <= indexEnd; j++)
      {
        task.inputsDelta[j] = k;
      }
      LWPR_Object model = learningInitModelLWPR(task);
      LearningData data = learningInit(task);
      learningDoLearn(data, model);
      double score = learningDoTesting(data, model);
      if (bestScore < 0.0 || bestScore > score)
      {
        bestScore = score;
        bestDelta = k;
      }
      std::cout << "Optimizing delta=" << k << " score=" << score << std::endl;
    }
    for (size_t j = indexBegin; j <= indexEnd; j++)
    {
      task.inputsDelta[j] = bestDelta;
    }
  }
  // Display found stats
  for (size_t i = indexBegin; i <= indexEnd; i++)
  {
    std::cout << "Inputs: " << task.inputs[i] << " bestLag=" << task.inputsLag[i] << std::endl;
    std::cout << "Inputs: " << task.inputs[i] << " bestDelta=" << task.inputsDelta[i] << std::endl;
  }
}
void learningOptimize(LearningTask& task, std::vector<libgp::GaussianProcess>& models)
{
  LearningData data = learningInit(task);
  learningDoLearn(data, models);
  learningTestStats(data, models);

  /*
  Leph::Plot plot;
  for (size_t i=0;i<data.logs.size();i++) {
      for (size_t j=0;j<data.logs[i].size();j++) {
          Eigen::VectorXd in = learningRetrieveInputs(task, data.logs[i], j);
          if (in.size() == 0) continue;
          plot.add(Leph::VectorLabel(
              "t", data.logs[i][j]("t"),
              "target", data.logs[i][j](task.outputs.front()),
              "fitted", model.f(in),
              "var+", model.f(in)+sqrt(model.var(in)),
              "var-", model.f(in)-sqrt(model.var(in))
          ));
      }
      plot.plot("t", "all").render();
      plot.clear();
  }
  */
}

static std::vector<std::string> trunkPosIKDOFs = {
  "trunk_pos_x",
  "trunk_pos_y",
  "trunk_pos_z",
};
static std::vector<std::string> trunkAxisIKDOFs = {
  "trunk_axis_x",
  "trunk_axis_y",
  "trunk_axis_z",
};
static std::vector<std::string> footPosIKDOFs = {
  "foot_pos_x",
  "foot_pos_y",
  "foot_pos_z",
};
static std::vector<std::string> footAxisIKDOFs = { "foot_axis_x", "foot_axis_y", "foot_axis_z" };

/**
 * Return negative value if success. Else return
 * the time of fisrt error.
 */
double checkIKErrors(const Leph::SplineContainer<Leph::CubicSpline>& splines)
{
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  for (double t = splines.min(); t < splines.max(); t += 0.02)
  {
    // Compute spline state
    Eigen::Vector3d trunkPos;
    Eigen::Vector3d trunkAxis;
    Eigen::Vector3d footPos;
    Eigen::Vector3d footAxis;
    trunkPos.x() = splines.get("trunk_pos_x").pos(t);
    trunkPos.y() = splines.get("trunk_pos_y").pos(t);
    trunkPos.z() = splines.get("trunk_pos_z").pos(t);
    trunkAxis.x() = splines.get("trunk_axis_x").pos(t);
    trunkAxis.y() = splines.get("trunk_axis_y").pos(t);
    trunkAxis.z() = splines.get("trunk_axis_z").pos(t);
    footPos.x() = splines.get("foot_pos_x").pos(t);
    footPos.y() = splines.get("foot_pos_y").pos(t);
    footPos.z() = splines.get("foot_pos_z").pos(t);
    footAxis.x() = splines.get("foot_axis_x").pos(t);
    footAxis.y() = splines.get("foot_axis_y").pos(t);
    footAxis.z() = splines.get("foot_axis_z").pos(t);
    // Compute IK
    bool isSuccess = model.trunkFootIK(Leph::HumanoidFixedModel::LeftSupportFoot, trunkPos,
                                       Leph::AxisToMatrix(trunkAxis), footPos, Leph::AxisToMatrix(footAxis));
    if (!isSuccess)
    {
      return t;
    }
  }

  return -1.0;
}

/**
 * Compute and insert in given logs container
 * model data and goal trajectory from splines
 */
void computeModelData(Leph::MatrixLabel& logs, const Leph::SplineContainer<Leph::CubicSpline>& splines)
{
  Leph::HumanoidFixedModel modelSpline(Leph::SigmabanModel);
  Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);
  Leph::HumanoidFixedPressureModel modelPos(Leph::SigmabanModel);
  for (size_t index = 1; index < logs.size(); index++)
  {
    double t = logs[index - 1]("t");
    // Compute spline state
    Eigen::Vector3d trunkPos;
    Eigen::Vector3d trunkAxis;
    Eigen::Vector3d footPos;
    Eigen::Vector3d footAxis;
    trunkPos.x() = splines.get("trunk_pos_x").pos(t);
    trunkPos.y() = splines.get("trunk_pos_y").pos(t);
    trunkPos.z() = splines.get("trunk_pos_z").pos(t);
    trunkAxis.x() = splines.get("trunk_axis_x").pos(t);
    trunkAxis.y() = splines.get("trunk_axis_y").pos(t);
    trunkAxis.z() = splines.get("trunk_axis_z").pos(t);
    footPos.x() = splines.get("foot_pos_x").pos(t);
    footPos.y() = splines.get("foot_pos_y").pos(t);
    footPos.z() = splines.get("foot_pos_z").pos(t);
    footAxis.x() = splines.get("foot_axis_x").pos(t);
    footAxis.y() = splines.get("foot_axis_y").pos(t);
    footAxis.z() = splines.get("foot_axis_z").pos(t);
    Eigen::Vector3d trunkPosVel;
    Eigen::Vector3d trunkAxisVel;
    Eigen::Vector3d footPosVel;
    Eigen::Vector3d footAxisVel;
    trunkPosVel.x() = splines.get("trunk_pos_x").vel(t);
    trunkPosVel.y() = splines.get("trunk_pos_y").vel(t);
    trunkPosVel.z() = splines.get("trunk_pos_z").vel(t);
    trunkAxisVel.x() = splines.get("trunk_axis_x").vel(t);
    trunkAxisVel.y() = splines.get("trunk_axis_y").vel(t);
    trunkAxisVel.z() = splines.get("trunk_axis_z").vel(t);
    footPosVel.x() = splines.get("foot_pos_x").vel(t);
    footPosVel.y() = splines.get("foot_pos_y").vel(t);
    footPosVel.z() = splines.get("foot_pos_z").vel(t);
    footAxisVel.x() = splines.get("foot_axis_x").vel(t);
    footAxisVel.y() = splines.get("foot_axis_y").vel(t);
    footAxisVel.z() = splines.get("foot_axis_z").vel(t);
    Eigen::Vector3d trunkPosAcc;
    Eigen::Vector3d trunkAxisAcc;
    Eigen::Vector3d footPosAcc;
    Eigen::Vector3d footAxisAcc;
    trunkPosAcc.x() = splines.get("trunk_pos_x").acc(t);
    trunkPosAcc.y() = splines.get("trunk_pos_y").acc(t);
    trunkPosAcc.z() = splines.get("trunk_pos_z").acc(t);
    trunkAxisAcc.x() = splines.get("trunk_axis_x").acc(t);
    trunkAxisAcc.y() = splines.get("trunk_axis_y").acc(t);
    trunkAxisAcc.z() = splines.get("trunk_axis_z").acc(t);
    footPosAcc.x() = splines.get("foot_pos_x").acc(t);
    footPosAcc.y() = splines.get("foot_pos_y").acc(t);
    footPosAcc.z() = splines.get("foot_pos_z").acc(t);
    footAxisAcc.x() = splines.get("foot_axis_x").acc(t);
    footAxisAcc.y() = splines.get("foot_axis_y").acc(t);
    footAxisAcc.z() = splines.get("foot_axis_z").acc(t);
    // Compute spline DOF
    bool isSuccess = modelSpline.trunkFootIK(Leph::HumanoidFixedModel::LeftSupportFoot, trunkPos,
                                             Leph::AxisToMatrix(trunkAxis), footPos, Leph::AxisToMatrix(footAxis));
    if (!isSuccess)
    {
      std::cout << "IK ERROR" << std::endl;
      exit(1);
    }
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      logs[index].append("spline_pos:" + name, modelSpline.get().getDOF(name));
    }
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      logs[index].append("spline_model:" + name, splines.get(name).pos(t));
    }
    // Compute spline ZMP
    Eigen::VectorXd dq = modelSpline.trunkFootIKVel(trunkPosVel, Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel),
                                                    footPosVel, Leph::AxisDiffToAngularDiff(footAxis, footAxisVel));
    Eigen::VectorXd ddq =
        modelSpline.trunkFootIKAcc(dq, trunkPosVel, Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel), footPosVel,
                                   Leph::AxisDiffToAngularDiff(footAxis, footAxisVel), trunkPosAcc,
                                   Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisAcc), footPosAcc,
                                   Leph::AxisDiffToAngularDiff(footAxis, footAxisAcc));
    Eigen::Vector3d zmp = modelSpline.zeroMomentPoint("left_foot_tip", dq, ddq);
    logs[index].append("spline_model:zmp_x", zmp.x());
    logs[index].append("spline_model:zmp_y", zmp.y());
    // Compute joint torques
    Eigen::VectorXd torques = modelSpline.get().inverseDynamics(dq, ddq);
    logs[index].append("torque:left_hip_yaw", torques(modelSpline.get().getDOFIndex("left_hip_yaw")));
    logs[index].append("torque:left_hip_pitch", torques(modelSpline.get().getDOFIndex("left_hip_pitch")));
    logs[index].append("torque:left_hip_roll", torques(modelSpline.get().getDOFIndex("left_hip_roll")));
    logs[index].append("torque:left_knee", torques(modelSpline.get().getDOFIndex("left_knee")));
    logs[index].append("torque:left_ankle_pitch", torques(modelSpline.get().getDOFIndex("left_ankle_pitch")));
    logs[index].append("torque:left_ankle_roll", torques(modelSpline.get().getDOFIndex("left_ankle_roll")));
    logs[index].append("torque:right_hip_yaw", torques(modelSpline.get().getDOFIndex("right_hip_yaw")));
    logs[index].append("torque:right_hip_pitch", torques(modelSpline.get().getDOFIndex("right_hip_pitch")));
    logs[index].append("torque:right_hip_roll", torques(modelSpline.get().getDOFIndex("right_hip_roll")));
    logs[index].append("torque:right_knee", torques(modelSpline.get().getDOFIndex("right_knee")));
    logs[index].append("torque:right_ankle_pitch", torques(modelSpline.get().getDOFIndex("right_ankle_pitch")));
    logs[index].append("torque:right_ankle_roll", torques(modelSpline.get().getDOFIndex("right_ankle_roll")));
    // Assign DOF
    Leph::VectorLabel vectDOFGoal = logs[index].extract("goal").rename("goal", "");
    modelGoal.get().setDOF(vectDOFGoal, false);
    Leph::VectorLabel vectDOFPos = logs[index].extract("pos").rename("pos", "");
    modelPos.get().setDOF(vectDOFPos, false);
    // Compute joint errors
    Leph::VectorLabel vectDOFErrors = vectDOFPos;
    vectDOFErrors.subOp(vectDOFGoal);
    logs[index].mergeUnion(vectDOFErrors.rename("", "error"));
    // Assiging pressure
    modelPos.setPressure(logs[index]("pressure:weight"), logs[index]("pressure:left_ratio"),
                         logs[index]("pressure:right_ratio"), logs[index]("pressure:left_x"),
                         logs[index]("pressure:left_y"), logs[index]("pressure:right_x"),
                         logs[index]("pressure:right_y"));
    // Contraint the model on the ground
    modelGoal.updateBase();
    modelPos.updateBase();
    // Set IMU data for motors real model state
    modelPos.setOrientation(
        Eigen::AngleAxisd(logs[index]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(logs[index]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
        false);
    // Compute model goal
    logs[index].append("goal_model:trunk_pos_x", modelGoal.get().position("trunk", "left_foot_tip").x());
    logs[index].append("goal_model:trunk_pos_y", modelGoal.get().position("trunk", "left_foot_tip").y());
    logs[index].append("goal_model:trunk_pos_z", modelGoal.get().position("trunk", "left_foot_tip").z());
    logs[index].append("goal_model:foot_pos_x", modelGoal.get().position("right_foot_tip", "left_foot_tip").x());
    logs[index].append("goal_model:foot_pos_y", modelGoal.get().position("right_foot_tip", "left_foot_tip").y());
    logs[index].append("goal_model:foot_pos_z", modelGoal.get().position("right_foot_tip", "left_foot_tip").z());
    logs[index].append("goal_model:trunk_axis_x",
                       Leph::MatrixToAxis(modelGoal.get().orientation("trunk", "left_foot_tip").transpose()).x());
    logs[index].append("goal_model:trunk_axis_y",
                       Leph::MatrixToAxis(modelGoal.get().orientation("trunk", "left_foot_tip").transpose()).y());
    logs[index].append("goal_model:trunk_axis_z",
                       Leph::MatrixToAxis(modelGoal.get().orientation("trunk", "left_foot_tip").transpose()).z());
    logs[index].append(
        "goal_model:foot_axis_x",
        Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", "left_foot_tip").transpose()).x());
    logs[index].append(
        "goal_model:foot_axis_y",
        Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", "left_foot_tip").transpose()).y());
    logs[index].append(
        "goal_model:foot_axis_z",
        Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", "left_foot_tip").transpose()).z());
    // Compute model pos
    logs[index].append("model:com_x", modelPos.get().centerOfMass("left_foot_tip").x());
    logs[index].append("model:com_y", modelPos.get().centerOfMass("left_foot_tip").y());
    logs[index].append("model:com_z", modelPos.get().centerOfMass("left_foot_tip").z());
    logs[index].append("model:trunk_pos_x", modelPos.get().position("trunk", "left_foot_tip").x());
    logs[index].append("model:trunk_pos_y", modelPos.get().position("trunk", "left_foot_tip").y());
    logs[index].append("model:trunk_pos_z", modelPos.get().position("trunk", "left_foot_tip").z());
    logs[index].append("model:foot_pos_x", modelPos.get().position("right_foot_tip", "left_foot_tip").x());
    logs[index].append("model:foot_pos_y", modelPos.get().position("right_foot_tip", "left_foot_tip").y());
    logs[index].append("model:foot_pos_z", modelPos.get().position("right_foot_tip", "left_foot_tip").z());
    logs[index].append("model:trunk_axis_x",
                       Leph::MatrixToAxis(modelPos.get().orientation("trunk", "left_foot_tip").transpose()).x());
    logs[index].append("model:trunk_axis_y",
                       Leph::MatrixToAxis(modelPos.get().orientation("trunk", "left_foot_tip").transpose()).y());
    logs[index].append("model:trunk_axis_z",
                       Leph::MatrixToAxis(modelPos.get().orientation("trunk", "left_foot_tip").transpose()).z());
    logs[index].append(
        "model:foot_axis_x",
        Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", "left_foot_tip").transpose()).x());
    logs[index].append(
        "model:foot_axis_y",
        Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", "left_foot_tip").transpose()).y());
    logs[index].append(
        "model:foot_axis_z",
        Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", "left_foot_tip").transpose()).z());
  }
}

Eigen::VectorXd learningRetrieveInputsSplines(const LearningTask& task,
                                              const Leph::SplineContainer<Leph::CubicSpline>& splines, double t,
                                              double step)
{
  Eigen::VectorXd in(2 * task.inputs.size());
  for (size_t i = 0; i < task.inputs.size(); i++)
  {
    if (t < step * (task.inputsLag[i] + task.inputsDelta[i]))
    {
      return Eigen::VectorXd();
    }
    std::string name = task.inputs[i];
    name = name.substr(11);
    in(2 * i) = splines.get(name).pos(t - step * task.inputsLag[i]);
    in(2 * i + 1) = splines.get(name).pos(t - step * (task.inputsLag[i] + task.inputsDelta[i]));
  }
  return in;
}

void testLearning()
{
  std::string splinesfile = "../../These/Data/logs-2016-02-02/trajBest_3.000000_1.000000_0.000000_2.398569.splines";
  Leph::SplineContainer<Leph::CubicSpline> splines;
  splines.importData(splinesfile);

  LearningTask task;
  task.logfiles = {
    /*
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-27-55.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-28-01.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-28-05.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-28-10.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-28-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-28-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-28-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-29-06.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-29-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-29-33.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-29-40.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-29-53.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-30-05.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-30-13.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-30-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-31-14.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-31-24.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-31-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-31-34.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-31-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-31-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-32-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-32-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-32-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-32-37.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-32-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-33-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-33-13.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-33-20.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-33-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-33-41.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-33-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-33-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-34-06.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-34-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-34-27.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-34-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-34-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-35-01.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-35-08.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-35-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-35-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-35-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-35-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-36-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-36-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-36-27.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-36-56.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-37-09.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-37-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-37-37.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-38-02.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-38-18.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-38-31.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-38-42.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-38-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-39-14.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-39-24.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-39-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-40-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-40-41.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-40-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-41-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-41-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-41-46.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-43-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-43-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-44-13.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-44-24.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-44-31.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-44-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-44-53.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-45-31.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-45-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-45-46.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-45-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-46-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-46-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-46-59.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-48-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-49-31.log",
    */
    "/tmp/a/model_2016-02-17-12-56-11.log", "/tmp/a/model_2016-02-17-12-56-15.log",
    "/tmp/a/model_2016-02-17-12-56-20.log", "/tmp/a/model_2016-02-17-12-56-28.log",
    "/tmp/a/model_2016-02-17-12-56-32.log", "/tmp/a/model_2016-02-17-12-56-38.log",
    "/tmp/a/model_2016-02-17-12-56-44.log", "/tmp/a/model_2016-02-17-12-56-49.log",
    "/tmp/a/model_2016-02-17-12-56-55.log", "/tmp/a/model_2016-02-17-12-57-00.log",
    "/tmp/a/model_2016-02-17-12-57-06.log", "/tmp/a/model_2016-02-17-12-57-11.log",
    "/tmp/a/model_2016-02-17-12-57-16.log", "/tmp/a/model_2016-02-17-12-57-21.log",
    "/tmp/a/model_2016-02-17-12-57-26.log", "/tmp/a/model_2016-02-17-12-57-31.log",
    "/tmp/a/model_2016-02-17-12-57-36.log", "/tmp/a/model_2016-02-17-12-57-41.log",
    "/tmp/a/model_2016-02-17-12-57-46.log", "/tmp/a/model_2016-02-17-12-57-51.log",
    "/tmp/a/model_2016-02-17-12-57-58.log", "/tmp/a/model_2016-02-17-12-58-03.log",
    "/tmp/a/model_2016-02-17-12-58-09.log", "/tmp/a/model_2016-02-17-12-58-14.log",
    "/tmp/a/model_2016-02-17-12-58-19.log", "/tmp/a/model_2016-02-17-12-58-25.log",
    "/tmp/a/model_2016-02-17-12-58-30.log", "/tmp/a/model_2016-02-17-12-58-35.log",
    "/tmp/a/model_2016-02-17-12-58-39.log", "/tmp/a/model_2016-02-17-12-58-44.log",
    "/tmp/a/model_2016-02-17-12-58-49.log", "/tmp/a/model_2016-02-17-12-58-54.log",
    "/tmp/a/model_2016-02-17-12-58-59.log", "/tmp/a/model_2016-02-17-12-59-04.log",
    "/tmp/a/model_2016-02-17-12-59-09.log", "/tmp/a/model_2016-02-17-12-59-22.log",
    "/tmp/a/model_2016-02-17-12-59-27.log", "/tmp/a/model_2016-02-17-12-59-33.log",
    "/tmp/a/model_2016-02-17-12-59-39.log", "/tmp/a/model_2016-02-17-12-59-44.log",
    "/tmp/a/model_2016-02-17-12-59-50.log", "/tmp/a/model_2016-02-17-12-59-55.log",
    "/tmp/a/model_2016-02-17-13-00-00.log", "/tmp/a/model_2016-02-17-13-00-06.log",
    "/tmp/a/model_2016-02-17-13-00-12.log", "/tmp/a/model_2016-02-17-13-00-18.log",
    "/tmp/a/model_2016-02-17-13-00-23.log", "/tmp/a/model_2016-02-17-13-00-28.log",
    "/tmp/a/model_2016-02-17-13-00-33.log", "/tmp/a/model_2016-02-17-13-00-38.log",
    "/tmp/a/model_2016-02-17-13-00-43.log", "/tmp/a/model_2016-02-17-13-00-48.log",
    "/tmp/a/model_2016-02-17-13-00-53.log",
  };
  auto engine = std::default_random_engine{};
  std::shuffle(std::begin(task.logfiles), std::end(task.logfiles), engine);
  task.inputs = {
    "goal_model:trunk_pos_x",  "goal_model:trunk_pos_y",  "goal_model:trunk_pos_z",
    "goal_model:trunk_axis_x", "goal_model:trunk_axis_y", "goal_model:trunk_axis_z",
    "goal_model:foot_pos_x",   "goal_model:foot_pos_y",   "goal_model:foot_pos_z",
  };
  task.outputs = {
    "pressure:left_x",
    "pressure:left_y",
  };
  task.inputsLag = std::vector<size_t>(task.inputs.size(), 8 /*XXX*/);
  task.inputsDelta = std::vector<size_t>(task.inputs.size(), 4 /*XXX*/);
  task.funcComputeModelData = [&splines](Leph::MatrixLabel& logs) { computeModelData(logs, splines); };

  // Do model learning
  LearningData data = learningInit(task);
  std::vector<libgp::GaussianProcess> models = learningInitModelGP(task);
  learningDoLearn(data, models);
  learningTestStats(data, models);
  // Declare optimization
  // std::vector<std::string> optimSplines = {"trunk_pos_x", "trunk_pos_y", "trunk_axis_x", "trunk_axis_y"};
  std::vector<std::string> optimSplines = { "trunk_pos_y" };
  // Subdivised splines
  for (size_t i = 0; i < optimSplines.size(); i++)
  {
    splines.get(optimSplines[i]).subdivide(1);
  }
  size_t optimPointSize = splines.get(optimSplines.front()).points().size() - 2;

  // Plot initial pressure
  Leph::Plot plot;
  for (double t = splines.min(); t < splines.max(); t += 0.02)
  {
    Eigen::VectorXd in = learningRetrieveInputsSplines(task, splines, t, 0.02);
    if (in.size() == 0)
    {
      continue;
    }
    for (size_t k = 0; k < models.size(); k++)
    {
      double val = models[k].f(in);
      plot.add(Leph::VectorLabel("t", t, "Init " + task.outputs[k], val));
    }
  }

  // Initial point
  Eigen::VectorXd pointsInit(2 * optimPointSize * optimSplines.size());
  for (size_t l = 0; l < optimSplines.size(); l++)
  {
    for (size_t k = 0; k < optimPointSize; k++)
    {
      pointsInit(2 * l * optimPointSize + 2 * k) = splines.get(optimSplines[l]).points()[k + 1].position;
      pointsInit(2 * l * optimPointSize + 2 * k + 1) = splines.get(optimSplines[l]).points()[k + 1].velocity;
    }
  }
  std::cout << "Parameters size=" << pointsInit.size() << std::endl;
  std::cout << "Parameters: " << pointsInit.transpose() << std::endl;
  // CMA-ES config
  libcmaes::CMAParameters<> cmaparams(pointsInit, -1.0);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(false);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_restarts(2);
  cmaparams.set_max_iter(500);
  cmaparams.set_elitism(true);
  // Fitness function
  libcmaes::FitFuncEigen fitness = [&splines, &models, &task, &pointsInit, &optimPointSize,
                                    &optimSplines](const Eigen::VectorXd& params) {
    if ((params - pointsInit).lpNorm<Eigen::Infinity>() > 0.05)
    {
      return 1000.0 + 100.0 * ((params - pointsInit).lpNorm<Eigen::Infinity>() - 0.05);
    }
    for (size_t l = 0; l < optimSplines.size(); l++)
    {
      for (size_t k = 0; k < optimPointSize; k++)
      {
        splines.get(optimSplines[l]).points()[k + 1].position = params(2 * l * optimPointSize + 2 * k);
        splines.get(optimSplines[l]).points()[k + 1].velocity = params(2 * l * optimPointSize + 2 * k + 1);
      }
      splines.get(optimSplines[l]).computeSplines();
    }
    double isError = checkIKErrors(splines);
    if (isError >= 0.0)
    {
      return 100.0 + 10.0 * (splines.max() - isError);
    }
    double score = 0.0;
    double valMax = -1.0;
    for (double t = splines.min(); t < splines.max(); t += 0.02)
    {
      Eigen::VectorXd in = learningRetrieveInputsSplines(task, splines, t, 0.02);
      if (in.size() == 0)
      {
        continue;
      }
      for (size_t k = 0; k < models.size(); k++)
      {
        // TODO use var()
        double val = models[k].f(in);
        if (valMax < 0.0 || valMax < fabs(val))
        {
          valMax = fabs(val);
        }
        score += 0.02 * fabs(val);
      }
    }
    score += 10.0 * valMax;
    return score;
  };
  // Do opimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
  Eigen::VectorXd bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
  // Assign best found splines parameters
  for (size_t l = 0; l < optimSplines.size(); l++)
  {
    for (size_t k = 0; k < optimPointSize; k++)
    {
      splines.get(optimSplines[l]).points()[k + 1].position = bestParams(2 * l * optimPointSize + 2 * k);
      splines.get(optimSplines[l]).points()[k + 1].velocity = bestParams(2 * l * optimPointSize + 2 * k + 1);
    }
    splines.get(optimSplines[l]).computeSplines();
  }

  for (double t = splines.min(); t < splines.max(); t += 0.02)
  {
    Eigen::VectorXd in = learningRetrieveInputsSplines(task, splines, t, 0.02);
    if (in.size() == 0)
    {
      continue;
    }
    for (size_t k = 0; k < models.size(); k++)
    {
      double val = models[k].f(in);
      plot.add(Leph::VectorLabel("t", t, "cmaes " + task.outputs[k], val));
    }
  }
  plot.plot("t", "all").render();

  splines.exportData("/tmp/cmaes.splines");
}

void testLearningClosedLoop()
{
  std::string splinesfile = "../../These/Data/logs-2016-02-02/newTrajs/"
                            "trajBest_3.000000_1.000000_0.000000_3.353406.splines";
  Leph::SplineContainer<Leph::CubicSpline> splines;
  splines.importData(splinesfile);

  LearningTask task;
  task.logfiles = {
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-02.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-59.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-03.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-08.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-13.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-18.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-22.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-35.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-35.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-52.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-56.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-04.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-08.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-25.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-28.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-36.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-40.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-34.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-42.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-46.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-20.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-25.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-33.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-43.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-53.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-02.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-15.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-22.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-01.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-20.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-24.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-55.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-05.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-10.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-23.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-36.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-41.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-47.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-52.log",
  };
  auto engine = std::default_random_engine{};
  std::shuffle(std::begin(task.logfiles), std::end(task.logfiles), engine);
  task.inputs = {
    "goal_model:trunk_pos_x",  "goal_model:trunk_pos_y",  "goal_model:trunk_pos_z",
    "goal_model:trunk_axis_x", "goal_model:trunk_axis_y", "goal_model:trunk_axis_z",
    "goal_model:foot_pos_x",   "goal_model:foot_pos_y",   "goal_model:foot_pos_z",
    /*
    "model:trunk_pos_x",
    "model:trunk_pos_y",
    "model:trunk_pos_z",
    "model:trunk_axis_x",
    "model:trunk_axis_y",
    "model:trunk_axis_z",
    "model:foot_pos_x",
    "model:foot_pos_y",
    "model:foot_pos_z",
    */
    /*
    "pressure:left_x",
    "pressure:left_y",
    */
  };
  task.outputs = {
    "pressure:left_x",
    "pressure:left_y",
  };
  task.inputsLag = std::vector<size_t>(task.inputs.size(), 8 /*XXX*/);
  task.inputsDelta = std::vector<size_t>(task.inputs.size(), 2 /*XXX*/);
  task.funcComputeModelData = [&splines](Leph::MatrixLabel& logs) { computeModelData(logs, splines); };

  /*
  while (task.logfiles.size() > 2) {
      LearningData data = learningInit(task);
      LWPR_Object model = learningInitModelLWPR(task);
      learningDoLearn(data, model);
      std::cout << task.logfiles.size() << " " << learningDoTesting(data, model) << std::endl;
      task.logfiles.pop_back();
  }
  return;
  */

  // Do model learning
  // learningOptimize(task, 0, 8);
  // learningOptimize(task, 9, 17);
  // learningOptimize(task, 0, 19);
  LearningData data = learningInit(task);
  LWPR_Object model = learningInitModelLWPR(task);
  learningDoLearn(data, model, false);
  learningTestStats(data, model);

  /*
  {
      Leph::Plot plot;
      for (size_t i=0;i<data.logs.size();i++) {
          for (size_t j=0;j<data.logs[i].size();j++) {
              Eigen::VectorXd in = learningRetrieveInputs(task, data.logs[i], j);
              if (in.size() == 0) continue;
              Eigen::VectorXd out = model.predict(in, 0.0);
              plot.add(Leph::VectorLabel(
                  "t", data.logs[i][j]("t"),
                  "target x", data.logs[i][j](task.outputs[0]),
                  "target y", data.logs[i][j](task.outputs[1]),
                  "fitted x", out(0),
                  "fitted y", out(1)
              ));
          }
          plot.plot("t", "all").render();
          plot.clear();
      }
  }
  */

  // return;

  // XXX XXXXX XXXXXXXX XXX XXXXXXXXX XXX XXXXXXXXXXXXX XXX XXXXXXXXXXXXXX XXX XXXXXXXXXXXXXXX XXX XXXXXXXXXXXXX XXX XXX
  // Declare optimization
  std::vector<std::string> optimSplines = { "trunk_pos_y" };
  // Subdivised splines
  for (size_t i = 0; i < optimSplines.size(); i++)
  {
    splines.get(optimSplines[i]).subdivide(1);
  }
  size_t optimPointSize = splines.get(optimSplines.front()).points().size() - 2;

  // Plot initial pressure
  Leph::Plot plot;
  for (double t = splines.min(); t < splines.max(); t += 0.02)
  {
    Eigen::VectorXd in = learningRetrieveInputsSplines(task, splines, t, 0.02);
    if (in.size() == 0)
    {
      continue;
    }
    Eigen::VectorXd val = model.predict(in, 0.0);
    plot.add(Leph::VectorLabel("t", t, "Init " + task.outputs[0], val(0), "Init " + task.outputs[1], val(1)));
  }

  // Initial point
  Eigen::VectorXd pointsInit(2 * optimPointSize * optimSplines.size());
  for (size_t l = 0; l < optimSplines.size(); l++)
  {
    for (size_t k = 0; k < optimPointSize; k++)
    {
      pointsInit(2 * l * optimPointSize + 2 * k) = splines.get(optimSplines[l]).points()[k + 1].position;
      pointsInit(2 * l * optimPointSize + 2 * k + 1) = splines.get(optimSplines[l]).points()[k + 1].velocity;
    }
  }
  std::cout << "Parameters size=" << pointsInit.size() << std::endl;
  std::cout << "Parameters: " << pointsInit.transpose() << std::endl;
  // CMA-ES config
  libcmaes::CMAParameters<> cmaparams(pointsInit, -1.0);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(false);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_restarts(2);
  cmaparams.set_max_iter(500);
  cmaparams.set_elitism(false);
  // Fitness function
  libcmaes::FitFuncEigen fitness = [&splines, &model, &task, &pointsInit, &optimPointSize,
                                    &optimSplines](const Eigen::VectorXd& params) {
    if ((params - pointsInit).lpNorm<Eigen::Infinity>() > 0.1)
    {
      return 1000.0 + 100.0 * ((params - pointsInit).lpNorm<Eigen::Infinity>() - 0.1);
    }
    for (size_t l = 0; l < optimSplines.size(); l++)
    {
      for (size_t k = 0; k < optimPointSize; k++)
      {
        splines.get(optimSplines[l]).points()[k + 1].position = params(2 * l * optimPointSize + 2 * k);
        splines.get(optimSplines[l]).points()[k + 1].velocity = params(2 * l * optimPointSize + 2 * k + 1);
      }
      splines.get(optimSplines[l]).computeSplines();
    }
    double isError = checkIKErrors(splines);
    if (isError >= 0.0)
    {
      return 100.0 + 100.0 * (splines.max() - isError);
    }
    double score = 0.0;
    double valMax = -1.0;
    for (double t = splines.min(); t < splines.max(); t += 0.02)
    {
      Eigen::VectorXd in = learningRetrieveInputsSplines(task, splines, t, 0.02);
      if (in.size() == 0)
      {
        continue;
      }
      Eigen::VectorXd val = model.predict(in, 0.0);
      if (valMax < 0.0 || valMax < fabs(val(0)))
      {
        valMax = fabs(val(0));
      }
      score += 0.02 * fabs(val(0));
      if (valMax < 0.0 || valMax < fabs(val(1)))
      {
        valMax = fabs(val(1));
      }
      score += 0.02 * fabs(val(1));
    }
    score += 2.0 * valMax;
    return score;
  };
  // Do opimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
  Eigen::VectorXd bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
  // Assign best found splines parameters
  for (size_t l = 0; l < optimSplines.size(); l++)
  {
    for (size_t k = 0; k < optimPointSize; k++)
    {
      splines.get(optimSplines[l]).points()[k + 1].position = bestParams(2 * l * optimPointSize + 2 * k);
      splines.get(optimSplines[l]).points()[k + 1].velocity = bestParams(2 * l * optimPointSize + 2 * k + 1);
    }
    splines.get(optimSplines[l]).computeSplines();
  }

  for (double t = splines.min(); t < splines.max(); t += 0.02)
  {
    Eigen::VectorXd in = learningRetrieveInputsSplines(task, splines, t, 0.02);
    if (in.size() == 0)
    {
      continue;
    }
    Eigen::VectorXd val = model.predict(in, 0.0);
    plot.add(Leph::VectorLabel("t", t, "cmaes " + task.outputs[0], val(0), "cmaes " + task.outputs[1], val(1)));
  }
  plot.plot("t", "all").render();

  splines.exportData("/tmp/cmaes.splines");
}

void testReParametrization()
{
  // Trajectory splines
  std::string splinesfile = "../../These/Data/logs-2016-02-02/trajBest_3.000000_1.000000_0.000000_2.398569.splines";
  Leph::SplineContainer<Leph::CubicSpline> splines;
  splines.importData(splinesfile);
  Leph::MatrixLabel logs;
  logs.load("../../These/Data/logs-2016-02-02/tmpTrajLog6/model_2016-02-17-11-27-55.log");
  computeModelData(logs, splines);

  Leph::SplineContainer<Leph::FittedSpline> container;
  for (const std::string& name : Leph::NamesCart)
  {
    container.add(name);
  }
  for (size_t i = 1; i < logs.size(); i++)
  {
    double t = logs[i - 1]("t");
    for (const std::string& name : Leph::NamesCart)
    {
      container.get(name).addPoint(t, logs[i]("model:" + name));
    }
  }

  /*
  for (size_t k=3;k<=6;k++) {
  for (size_t l=k+1;l<=20;l++) {
  */
  for (const std::string& name : Leph::NamesCart)
  {
    // container.get(name).fittingGlobal(k, l);
    container.get(name).fittingGlobal(3, 9);
  }

  Leph::Plot plot;
  for (size_t i = 1; i < logs.size(); i++)
  {
    double t = logs[i - 1]("t");
    for (const std::string& name : Leph::NamesCart)
    {
      plot.add(Leph::VectorLabel("t", t, "target " + name, logs[i]("model:" + name), "fitted " + name,
                                 container.get(name).pos(t)));
    }
  }
  plot.plot("t", "all").render();

  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  double score = 0.0;
  for (size_t i = 1; i < logs.size(); i++)
  {
    double t = logs[i - 1]("t");
    // Compute current positions
    Eigen::Vector3d trunkPos = Eigen::Vector3d(container.get("trunk_pos_x").pos(t), container.get("trunk_pos_y").pos(t),
                                               container.get("trunk_pos_z").pos(t));
    Eigen::Vector3d trunkAxisAngles =
        Eigen::Vector3d(container.get("trunk_axis_x").pos(t), container.get("trunk_axis_y").pos(t),
                        container.get("trunk_axis_z").pos(t));
    Eigen::Vector3d footPos = Eigen::Vector3d(container.get("foot_pos_x").pos(t), container.get("foot_pos_y").pos(t),
                                              container.get("foot_pos_z").pos(t));
    Eigen::Vector3d footAxisAngles = Eigen::Vector3d(
        container.get("foot_axis_x").pos(t), container.get("foot_axis_y").pos(t), container.get("foot_axis_z").pos(t));
    if (trunkPos.norm() > 1.0 || trunkAxisAngles.norm() > M_PI / 2.0 || footPos.norm() > 1.0 ||
        footAxisAngles.norm() > M_PI / 2.0)
    {
      std::cout << "FITTING ERROR t=" << t << std::endl;
      continue;
    }
    // Compute current velocities
    Eigen::Vector3d trunkPosVel = Eigen::Vector3d(
        container.get("trunk_pos_x").vel(t), container.get("trunk_pos_y").vel(t), container.get("trunk_pos_z").vel(t));
    Eigen::Vector3d trunkAxisAnglesVel =
        Eigen::Vector3d(container.get("trunk_axis_x").vel(t), container.get("trunk_axis_y").vel(t),
                        container.get("trunk_axis_z").vel(t));
    Eigen::Vector3d footPosVel = Eigen::Vector3d(container.get("foot_pos_x").vel(t), container.get("foot_pos_y").vel(t),
                                                 container.get("foot_pos_z").vel(t));
    Eigen::Vector3d footAxisAnglesVel = Eigen::Vector3d(
        container.get("foot_axis_x").vel(t), container.get("foot_axis_y").vel(t), container.get("foot_axis_z").vel(t));
    // Compute current accelerations
    Eigen::Vector3d trunkPosAcc = Eigen::Vector3d(
        container.get("trunk_pos_x").acc(t), container.get("trunk_pos_y").acc(t), container.get("trunk_pos_z").acc(t));
    Eigen::Vector3d trunkAxisAnglesAcc =
        Eigen::Vector3d(container.get("trunk_axis_x").acc(t), container.get("trunk_axis_y").acc(t),
                        container.get("trunk_axis_z").acc(t));
    Eigen::Vector3d footPosAcc = Eigen::Vector3d(container.get("foot_pos_x").acc(t), container.get("foot_pos_y").acc(t),
                                                 container.get("foot_pos_z").acc(t));
    Eigen::Vector3d footAxisAnglesAcc = Eigen::Vector3d(
        container.get("foot_axis_x").acc(t), container.get("foot_axis_y").acc(t), container.get("foot_axis_z").acc(t));
    // Assign model state
    bool isSuccess =
        model.trunkFootIK(Leph::HumanoidFixedModel::LeftSupportFoot, trunkPos, Leph::AxisToMatrix(trunkAxisAngles),
                          footPos, Leph::AxisToMatrix(footAxisAngles));
    if (!isSuccess)
    {
      std::cout << "IK ERROR t=" << t << std::endl;
      continue;
    }
    // Compute dofs dq and ddq
    Eigen::VectorXd dq =
        model.trunkFootIKVel(trunkPosVel, Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesVel), footPosVel,
                             Leph::AxisDiffToAngularDiff(footAxisAngles, footAxisAnglesVel));
    Eigen::VectorXd ddq =
        model.trunkFootIKAcc(dq, trunkPosVel, Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesVel),
                             footPosVel, Leph::AxisDiffToAngularDiff(footAxisAngles, footAxisAnglesVel), trunkPosAcc,
                             Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesAcc), footPosAcc,
                             Leph::AxisDiffToAngularDiff(footAxisAngles, footAxisAnglesAcc));
    // Compute ZMP
    Eigen::Vector3d zmp = model.zeroMomentPoint("left_foot_tip", dq, ddq);
    // Compute fitting score
    score += 0.02 * fabs(zmp.x() - logs[i]("pressure:left_x"));
    score += 0.02 * fabs(zmp.y() - logs[i]("pressure:left_y"));
    // Plot
    plot.add(Leph::VectorLabel("t", t, "zmp x", zmp.x(), "zmp y", zmp.y(), "goal zmp x", logs[i]("spline_model:zmp_x"),
                               "goal zmp y", logs[i]("spline_model:zmp_y"), "pressure x", logs[i]("pressure:left_x"),
                               "pressure y", logs[i]("pressure:left_y")));
  }
  plot.plot("t", "zmp x").plot("t", "goal zmp x").plot("t", "pressure x").render();
  plot.plot("t", "zmp y").plot("t", "goal zmp y").plot("t", "pressure y").render();
  plot.plot("t", "zmp x")
      .plot("t", "zmp y")
      .plot("t", "goal zmp x")
      .plot("t", "goal zmp y")
      .plot("t", "pressure x")
      .plot("t", "pressure y")
      .render();
  /*
  std::cout << "# " << k << " " << l << " --- " << score << std::endl;
  }
  }
  */
}

void testLearningMotorModel()
{
  std::string splinesfile = "../../These/Data/logs-2016-02-02/newTrajs/"
                            "trajBest_3.000000_1.000000_0.000000_3.353406.splines";
  Leph::SplineContainer<Leph::CubicSpline> splines;
  splines.importData(splinesfile);
  LearningTask task;
  task.logfiles = {
    //"/tmp/a/model_2016-02-23-18-56-15.log",
    /*
    "/tmp/a/model_2016-02-23-18-46-20.log",
    "/tmp/a/model_2016-02-23-18-24-50.log",
    "/tmp/a/model_2016-02-23-18-24-56.log",
    "/tmp/a/model_2016-02-23-18-24-59.log",
    "/tmp/a/model_2016-02-23-18-25-04.log",
    "/tmp/a/model_2016-02-23-18-25-20.log",
    "/tmp/a/model_2016-02-23-18-25-25.log",
    "/tmp/a/model_2016-02-23-18-25-30.log",
    "/tmp/a/model_2016-02-23-18-25-34.log",
    "/tmp/a/model_2016-02-23-18-25-38.log",
    "/tmp/a/model_2016-02-23-18-25-43.log",
    "/tmp/a/model_2016-02-23-18-25-50.log",
    "/tmp/a/model_2016-02-23-18-25-59.log",
    "/tmp/a/model_2016-02-23-18-26-08.log",
    "/tmp/a/model_2016-02-23-18-26-15.log",
    "/tmp/a/model_2016-02-23-18-26-21.log",
    "/tmp/a/model_2016-02-23-18-26-28.log",
    "/tmp/a/model_2016-02-23-18-26-35.log",
    */
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-02.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-59.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-03.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-08.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-13.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-18.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-22.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-35.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-30.log",
    /*
    //"../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-35.log",
    //"../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-52.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-56.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-04.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-08.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-25.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-28.log",
    //"../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-32.log",
    //"../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-36.log",
    //"../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-40.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-34.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-42.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-46.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-20.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-25.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-33.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-43.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-53.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-02.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-15.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-22.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-01.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-11.log",
    //"../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-20.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-24.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-55.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-05.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-10.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-23.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-36.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-41.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-47.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-52.log",
    */
  };
  auto engine = std::default_random_engine{};
  std::shuffle(std::begin(task.logfiles), std::end(task.logfiles), engine);
  task.inputs = {
    "goal:left_hip_pitch",
    "torque:left_hip_pitch",
    "error:left_hip_pitch",
  };
  task.outputs = {
    "error:left_hip_pitch",
  };
  task.inputsLag = std::vector<size_t>(task.inputs.size(), 0 /*XXX*/);
  task.inputsDelta = std::vector<size_t>(task.inputs.size(), 0 /*XXX*/);
  task.funcComputeModelData = [&splines](Leph::MatrixLabel& logs) { computeModelData(logs, splines); };

  LearningData data = learningInit(task);
  /*
  LWPR_Object model = learningInitModelLWPR(task);
  learningDoLearn(data, model, false);
  learningTestStats(data, model);
  */

  /*
  for (size_t i=0;i<data.logs.size();i++) {
  data.logs[i].plot()
      .plot("t", "pos:left_ankle_pitch")
      .plot("t", "goal:left_ankle_pitch")
      .plot("t", "torque:left_ankle_pitch")
      .plot("t", "spline_model:zmp_ix")
      .plot("t", "pressure:left_x")
      .render();
  }
  */
  for (size_t i = 0; i < data.logs.size(); i++)
  {
    data.logs[i]
        .plot()
        .plot("t", "pos:left_ankle_pitch")
        .plot("t", "goal:left_ankle_pitch")
        .plot("t", "torque:left_ankle_pitch")
        .render();
    data.logs[i]
        .plot()
        .plot("t", "pos:left_ankle_pitch")
        .plot("t", "pos:left_ankle_roll")
        .plot("t", "pos:left_knee")
        .plot("t", "pos:left_hip_pitch")
        .plot("t", "pos:left_hip_yaw")
        .plot("t", "pos:left_hip_roll")
        .render();
  }

  for (const std::string& name : Leph::NamesDOFLeg)
  {
    // std::string name = "right_knee";
    std::cout << "------------ " << name << std::endl;
    /*
    data.logs.front().plot()
        .plot("t", "pos:"+name)
        .plot("t", "goal:"+name)
        .render();
    */

    Leph::SimpleLinearRegression regression;
    for (size_t i = 0; i < data.logs.size(); i++)
    {
      for (size_t j = 10; j < data.logs[i].size(); j++)
      {
        Eigen::VectorXd in(2);
        in <<
            // 1.0,
            // data.logs[i][j-4]("goal:"+name),
            data.logs[i][j - 2]("goal:" + name),
            // pow(data.logs[i][j-1]("goal:"+name),2),
            // pow(data.logs[i][j]("goal:"+name),2),
            // data.logs[i][j-4]("torque:"+name),
            // data.logs[i][j-2]("torque:"+name),
            // data.logs[i][j-2]("pos:"+name),
            data.logs[i][j - 1]("pos:" + name);
        regression.add(in, data.logs[i][j]("pos:" + name));
      }
    }
    regression.regression();
    std::cout << regression.parameters().transpose() << std::endl;
    // regression.print();

    double score = 0.0;
    for (size_t i = 0; i < data.logs.size(); i++)
    {
      double pos = data.logs[i][2]("pos:" + name);
      // double posOld = pos;
      // Leph::Plot plot;
      for (size_t j = 10; j < data.logs[i].size(); j++)
      {
        Eigen::VectorXd in(2);
        in <<
            // 1.0,
            // data.logs[i][j-4]("goal:"+name),
            data.logs[i][j - 2]("goal:" + name),
            // pow(data.logs[i][j-1]("goal:"+name),2),
            // pow(data.logs[i][j]("goal:"+name),2),
            // data.logs[i][j-4]("torque:"+name),
            // data.logs[i][j-2]("torque:"+name),
            // posOld,
            pos;
        // posOld = pos;
        pos = regression.prediction(in);
        score += fabs(pos - data.logs[i][j]("pos:" + name));
        /*
        plot.add(Leph::VectorLabel(
            "t", data.logs[i][j]("t"),
            "target pos", data.logs[i][j]("pos:"+name),
            "target goal", data.logs[i][j]("goal:"+name),
            "predicted pos", pos
        ));
        */
      }
      // plot.plot("t", "all").render();
      // std::cout << i << " score:" << score << std::endl;
    }
    std::cout << "Score error: " << score << std::endl;
  }
}

void testLogsPush()
{
  std::vector<std::string> filenames = {
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-22-49.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-23-05.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-23-17.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-23-32.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-23-42.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-23-50.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-23-57.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-24-05.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-24-15.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-24-31.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-26-48.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-27-31.log",
    "/home/leph/Code/These/Data/logs-2016-03-17/model_2016-03-17-19-28-30.log",
  };
  std::vector<std::pair<size_t, size_t>> bounds;

  for (size_t i = 0; i < filenames.size(); i++)
  {
    Leph::MatrixLabel logs;
    logs.load(filenames[i]);
    bounds.push_back({ 0, logs.size() - 1 });
  }
  bounds[0] = { 457, 624 };
  bounds[1] = { 295, 487 };
  bounds[2] = { 88, 180 };
  bounds[3] = { 122, 300 };
  bounds[4] = { 138, 300 };
  bounds[5] = { 97, 240 };
  bounds[6] = { 69, 220 };
  bounds[7] = { 72, 250 };
  bounds[8] = { 83, 187 };
  bounds[9] = { 110, 180 };
  bounds[10] = { 0, 2974 };
  bounds[11] = { 0, 1368 };
  bounds[12] = { 0, 2200 };

  Leph::Plot plot;
  std::vector<Leph::MatrixLabel> container;
  for (size_t i = 6; i < filenames.size() - 6; i++)
  {
    Leph::MatrixLabel logs;
    logs.load(filenames[i]);
    std::cout << i << " " << bounds[i].first << " " << bounds[i].second << std::endl;
    /*
    logs.range(bounds[i].first, bounds[i].second).plot()
        .plot("index", "sensor:pitch")
        .plot("index", "pressure:left_x")
        .plot("index", "sensor:gyro_x")
        .plot("index", "sensor:gyro_y")
        .render();
    */
    container.push_back(logs.range(bounds[i].first, bounds[i].second));
    Leph::ComputeModelData(container.back());
    plot.merge(container.back().plot());
  }
  plot.plot("time:timestamp", "model:com_x")
      .plot("time:timestamp", "pos:base_pitch")
      .plot("time:timestamp", "pressure:left_x")
      .plot("time:timestamp", "sensor:pitch")
      .plot("time:timestamp", "model:trunk_pos_x")
      .plot("time:timestamp", "model:trunk_axis_y")
      .plot("time:timestamp", "goal_model:trunk_pos_x")
      //.plot("time:timestamp", "goal_model:trunk_axis_y")
      .render();
  plot.plot("model:com_x", "model:com_x_vel", "pos:base_pitch").render();

  /*
  auto lambda = [](const Leph::MatrixLabel& logs, size_t index, double last1Target, double last2Target) ->
  Eigen::VectorXd { Eigen::VectorXd in(21); in << 1.0, pow(last1Target, 2), pow(last1Target, 3), last1Target,
          pow(last2Target, 2),
          pow(last2Target, 3),
          last2Target,

          pow(logs[index-2]("model:trunk_pos_x"), 2),
          logs[index-2]("model:trunk_pos_x"),
          pow(logs[index]("model:trunk_pos_x"), 2),
          logs[index]("model:trunk_pos_x"),
          pow(logs[index-2]("model:trunk_axis_y"), 2),
          logs[index-2]("model:trunk_axis_y"),
          pow(logs[index]("model:trunk_axis_y"), 2),
          logs[index]("model:trunk_axis_y"),

          pow(logs[index]("model:com_x"), 2),
          pow(logs[index]("model:com_x"), 3),
          logs[index]("model:com_x"),
          pow(logs[index-2]("model:com_x"), 2),
          pow(logs[index-2]("model:com_x"), 3),
          logs[index-2]("model:com_x");
      return in;
  };

  Leph::SimpleLinearRegression regression;
  for (size_t i=0;i<container.size();i++) {
      for (size_t j=3;j<container[i].size();j++) {
          regression.add(
              lambda(container[i], j, container[i][j-1]("pos:base_pitch"), container[i][j-2]("pos:base_pitch")),
              container[i][j]("pos:base_pitch"));
      }
  }
  regression.regression();
  regression.print();

  for (size_t i=0;i<container.size();i++) {
      Leph::Plot plotReg;
      double target1Last = container[i][2]("pos:base_pitch");
      double target2Last = container[i][1]("pos:base_pitch");
      for (size_t j=3;j<container[i].size();j++) {
          double y = regression.prediction(lambda(container[i], j, target1Last, target2Last));
          plotReg.add(Leph::VectorLabel(
              "t", container[i][j]("time:timestamp"),
              "target", container[i][j]("pos:base_pitch"),
              "fitted", y
          ));
          target2Last = target1Last;
          target1Last = y;
      }
      plotReg.plot("t", "all").render();
  }
  */

  /*
  for (size_t i=0;i<container.size();i++) {
      double velPos =
          (container[i][4]("model:trunk_pos_x") - container[i][0]("model:trunk_pos_x")) /
          (container[i][4]("time:timestamp")/1000.0 - container[i][0]("time:timestamp")/1000.0);
      double velAxis =
          (container[i][4]("model:trunk_axis_y") - container[i][0]("model:trunk_axis_y")) /
          (container[i][4]("time:timestamp")/1000.0 - container[i][0]("time:timestamp")/1000.0);
      std::cout
          << container[i][0]("model:trunk_pos_x") << " " << velPos << " "
          << container[i][0]("model:trunk_axis_x") << " " << velAxis << " "
          << std::endl;
  }
  return;
  */

  // CMA-ES config
  Eigen::VectorXd initParams = 0.01 * Eigen::VectorXd::Ones(19);
  libcmaes::CMAParameters<> cmaparams(initParams, -1.0);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(false);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_restarts(4);
  cmaparams.set_max_iter(5000);
  cmaparams.set_elitism(true);
  bool doPlot = false;
  libcmaes::FitFuncEigen fitness = [&container, &doPlot](const Eigen::VectorXd& params) {
    double cost = 0.0;
    double maxError = 0.0;
    for (size_t i = 0; i < container.size(); i++)
    {
      Leph::Plot plotReg;
      double target1Last = container[i][2]("pos:base_pitch");
      double target2Last = container[i][1]("pos:base_pitch");
      for (size_t j = 3; j < container[i].size(); j++)
      {
        Eigen::VectorXd in(19);
        in << 1.0, target1Last, pow(target1Last, 2), pow(target1Last, 3), target2Last, pow(target2Last, 2),
            pow(target2Last, 3),

            container[i][j - 0]("model:trunk_axis_y"), pow(container[i][j - 0]("model:trunk_axis_y"), 2),
            pow(container[i][j - 0]("model:trunk_axis_y"), 3), container[i][j - 2]("model:trunk_axis_y"),
            pow(container[i][j - 2]("model:trunk_axis_y"), 2), pow(container[i][j - 2]("model:trunk_axis_y"), 3),

            container[i][j - 0]("model:trunk_pos_x"), pow(container[i][j - 0]("model:trunk_pos_x"), 2),
            pow(container[i][j - 0]("model:trunk_pos_x"), 3), container[i][j - 2]("model:trunk_pos_x"),
            pow(container[i][j - 2]("model:trunk_pos_x"), 2), pow(container[i][j - 2]("model:trunk_pos_x"), 3)

            /*
            container[i][j-2]("model:com_x"),
            pow(container[i][j-2]("model:com_x"), 2),
            pow(container[i][j-2]("model:com_x"), 3),
            pow(container[i][j]("model:com_x"), 2),
            pow(container[i][j]("model:com_x"), 3),
            container[i][j]("model:com_x")
            */
            ;
        double fitted = params.dot(in);
        double target = container[i][j]("pos:base_pitch");
        double error = pow(fitted - target, 2);
        if (error > maxError)
        {
          maxError = error;
        }
        cost += error;
        target2Last = target1Last;
        target1Last = fitted;
        if (doPlot)
        {
          plotReg.add(Leph::VectorLabel("t", container[i][j]("time:timestamp"), "target",
                                        container[i][j]("pos:base_pitch"), "fitted", fitted));
        }
      }
      if (doPlot)
      {
        std::cout << "MaxError: " << maxError << std::endl;
        plotReg.plot("t", "all").render();
      }
    }
    return cost;
  };
  // Do opimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
  Eigen::VectorXd bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
  std::cout << "BEST PARAMS: " << bestParams.transpose() << std::endl;
  doPlot = true;
  fitness(bestParams);
}

int main()
{
  testLogsPush();
  return 0;
  // testLearningMotorModel();
  // return 0;
  /*
  testLearningClosedLoop();
  return 0;
  testReParametrization();
  return 0;
  testLearning();
  return 0;
  */

  // Trajectory splines
  std::string splinesfile = "../../These/Data/logs-2016-02-02/trajBest_3.000000_1.000000_0.000000_2.398569.splines";
  // Loading splines
  Leph::SplineContainer<Leph::CubicSpline> splines;
  splines.importData(splinesfile);

  // Loading data
  Leph::MatrixLabel logs;
  std::vector<std::string> allLogsFiles = {
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-54-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-02.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-55-59.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-03.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-08.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-13.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-18.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-22.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-35.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-56-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-35.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-52.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-57-56.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-04.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-08.log",
    /*
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-25.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-28.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-36.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-40.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-49.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-58-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-12.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-21.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-26.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-30.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-34.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-42.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-46.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-50.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-54.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-18-59-58.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-20.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-25.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-33.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-43.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-48.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-53.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-00-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-02.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-15.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-22.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-38.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-44.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-01-57.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-01.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-07.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-11.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-16.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-20.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-24.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-29.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-39.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-45.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-51.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-02-55.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-00.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-05.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-10.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-17.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-23.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-32.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-36.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-41.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-47.log",
    "../../These/Data/logs-2016-02-02/tmpTrajLog8/model_2016-02-18-19-03-52.log",
    */
  };
  /*
  for (const std::string& name : allLogsFiles) {
      Leph::MatrixLabel tmpLogs;
      tmpLogs.load(name);
      double score = 0.0;
      for (size_t i=0;i<tmpLogs.size();i++) {
          score += fabs(tmpLogs[i]("pressure:left_x"));
          score += fabs(tmpLogs[i]("pressure:left_y"));
      }
      std::cout << name << " " << score << std::endl;
  }
  return 0;
  */
  for (const std::string& name : allLogsFiles)
  {
    logs.load(name);
  }

  // Compute model data
  computeModelData(logs, splines);

  Leph::Plot plot = logs.plot();

  plot.plot("t", "pos:left_hip_pitch", Leph::Plot::LinesPoints)
      .plot("t", "goal:left_hip_pitch", Leph::Plot::LinesPoints)
      .plot("t", "error:left_hip_pitch", Leph::Plot::LinesPoints)
      .plot("t", "torque:left_hip_pitch", Leph::Plot::LinesPoints)
      .render();
  plot.plot("goal:left_hip_pitch", "torque:left_hip_pitch", "error:left_hip_pitch", Leph::Plot::Points, "t").render();
  plot.plot("t", "pressure:left_x", Leph::Plot::LinesPoints, "index").render();
  plot.plot("t", "pressure:left_y", Leph::Plot::LinesPoints, "index").render();
  plot.plot("pressure:left_x", "pressure:left_y", Leph::Plot::LinesPoints, "t").render();
  plot.plot("goal_model:trunk_pos_x", "model:trunk_pos_x").render();
  plot.plot("t", "goal_model:trunk_pos_x").plot("t", "model:trunk_pos_x").render();

  for (const std::string& name : trunkPosIKDOFs)
  {
    plot.plot("t", "goal_model:" + name);
  }
  plot.render();
  for (const std::string& name : trunkPosIKDOFs)
  {
    plot.plot("t", "goal_model:" + name);
    plot.plot("t", "model:" + name);
  }
  plot.render();
  plot.plot("t", "pressure:left_x").plot("t", "pressure:left_y").render();
  plot.plot("t", "model:trunk_pos_x").plot("t", "goal_model:trunk_pos_x").plot("t", "pressure:left_x").render();
  plot.plot("t", "model:trunk_pos_x").plot("t", "model:foot_pos_x").plot("t", "pressure:left_x").render();
  plot.plot("model:trunk_pos_x", "model:foot_pos_x", "pressure:left_x").render();
  plot.plot("goal_model:trunk_pos_x", "model:trunk_pos_x", "pressure:left_x").render();
  plot.plot("goal_model:trunk_pos_y", "goal_model:trunk_pos_x", "model:trunk_pos_x").render();

  for (const std::string& name : trunkPosIKDOFs)
  {
    plot.plot("t", "goal_model:" + name);
    plot.plot("t", "model:" + name);
  }
  plot.render();
  for (const std::string& name : trunkAxisIKDOFs)
  {
    plot.plot("t", "goal_model:" + name);
    plot.plot("t", "model:" + name);
  }
  plot.render();
  for (const std::string& name : footPosIKDOFs)
  {
    plot.plot("t", "goal_model:" + name);
    plot.plot("t", "model:" + name);
  }
  plot.render();
  for (const std::string& name : footAxisIKDOFs)
  {
    plot.plot("t", "goal_model:" + name);
    plot.plot("t", "model:" + name);
  }
  plot.render();

  return 0;

  // Initialize model instances
  Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);
  Leph::HumanoidFixedPressureModel model(Leph::SigmabanModel);
  Leph::ModelViewer viewer(1200, 900);
  Leph::Scheduling scheduling(50.0);

  // Main viewer loop
  size_t indexLog = 0;
  Leph::VectorLabel vectDOF = logs[0].extract("pos").rename("pos", "");
  while (viewer.update())
  {
    // Assign DOF
    Leph::VectorLabel vectDOF = logs[indexLog].extract("pos").rename("pos", "");
    model.get().setDOF(vectDOF, false);
    // Assign splines goals
    Leph::VectorLabel vectDOFGoal = logs[indexLog].extract("spline_pos").rename("spline_pos", "");
    modelGoal.get().setDOF(vectDOFGoal, false);
    // Assiging pressure
    model.setPressure(logs[indexLog]("pressure:weight"), logs[indexLog]("pressure:left_ratio"),
                      logs[indexLog]("pressure:right_ratio"), logs[indexLog]("pressure:left_x"),
                      logs[indexLog]("pressure:left_y"), logs[indexLog]("pressure:right_x"),
                      logs[indexLog]("pressure:right_y"));
    // Contraint the model on the ground
    model.updateBase();
    // Set IMU data for motors real model state
    model.setOrientation(
        Eigen::AngleAxisd(logs[indexLog]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(logs[indexLog]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
        false);
    // Retrieve center of pressure
    Eigen::Vector3d copLeft = model.centerOfPressureLeft("origin");
    copLeft.z() = 0.0;
    // Display foot pressure force
    viewer.drawBox(0.005, 0.005, 0.1 * model.pressureLeftRatio(),
                   copLeft + Eigen::Vector3d(0, 0, 0.1 * model.pressureLeftRatio()), Eigen::Matrix3d::Identity(), 1.0,
                   0.0, 0.0);
    // Display pressure trajectory
    viewer.addTrackedPoint(copLeft, Leph::ModelViewer::Red);
    // Display
    Leph::ModelDraw(model.get(), viewer);
    Leph::ModelDraw(modelGoal.get(), viewer);
    // Scheduling
    scheduling.wait();
    // Index
    indexLog++;
    if (indexLog >= logs.size())
    {
      indexLog = 0;
    }
  }

  return 0;
}
