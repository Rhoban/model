#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <algorithm>
#include "TimeSeries/SeriesUtils.h"
#include "Utils/LWPRUtils.h"
#include "Plot/Plot.hpp"

/**
 * Statistical data
 */
struct Gaussian
{
  double mean;
  double var;
  double count;
};

/**
 * Compute the mean of given Gaussian
 */
static Gaussian mergeGaussian(const std::vector<Gaussian>& vect)
{
  Gaussian result;
  result.mean = 0.0;
  result.var = 0.0;
  result.count = 0.0;

  for (size_t i = 0; i < vect.size(); i++)
  {
    result.mean += vect[i].mean;
    result.var += pow(vect[i].mean, 2);
    result.count++;
  }
  if (result.count < 1.0)
  {
    throw std::logic_error("MergeGaussian empty");
  }
  result.mean = result.mean / (double)result.count;
  result.var = result.var / (double)result.count;
  result.var -= result.mean * result.mean;

  return result;
}

/**
 * Compute and return confidence bounds
 */
static double confidenceBounds(const Gaussian& g)
{
  return 1.96 * sqrt(g.var / g.count);
}

/**
 * Setup 3 models, and do lot of good things
 */
static void setUpModels(const Leph::MatrixLabel& logs, bool invModel, size_t beginIndex, size_t endIndex,
                        Leph::ModelSeries& modelWithMocap, Leph::ModelSeries& modelNoMocap,
                        Leph::ModelSeries& modelNoSensor, bool doPreLoad, bool doLearning, double timeLearning,
                        double timeTesting, bool isQuiet)
{
  // Initialize ModelSeries
  if (!isQuiet)
    std::cout << "Initiating ModelSeries" << std::endl;
  Leph::initModelSeries(modelWithMocap, true, true, true);
  Leph::initModelSeries(modelNoMocap, false, true, false);
  Leph::initModelSeries(modelNoSensor, false, false, true, true);

  // Load data into TimeSeries
  if (!isQuiet)
    std::cout << "Injecting data" << std::endl;
  if (beginIndex == (size_t)-1)
    beginIndex = 0;
  if (endIndex == (size_t)-1)
    endIndex = logs.size();
  for (size_t i = beginIndex; i < endIndex; i++)
  {
    double time = logs[i]("time:timestamp") / 1000.0;
    Leph::appendModelSeries(modelWithMocap, time, logs[i], invModel);
    Leph::appendModelSeries(modelNoMocap, time, logs[i], invModel);
    Leph::appendModelSeries(modelNoSensor, time, logs[i], invModel);
  }

  // Compute all values throught Concepts graph
  if (!isQuiet)
    std::cout << "Computing concepts" << std::endl;
  modelWithMocap.propagateConcepts();
  modelNoMocap.propagateConcepts();
  modelNoSensor.propagateConcepts();

  // Load regressions meta parameters
  modelWithMocap.regressionsParameterLoad("/tmp/");

  if (doPreLoad)
  {
    modelWithMocap.regressionsLoad("/tmp/");
  }

  // Learn Regression models
  if (doLearning)
  {
    if (!isQuiet)
      std::cout << "Learning regressions" << std::endl;
    double beginTime = modelWithMocap.series("mocap_x").timeMin();
    double endTime = modelWithMocap.series("mocap_x").timeMax();
    double beginLearnTime = beginTime + 10.0;
    double endLearnTime = timeLearning;
    if (timeLearning < 0.0)
    {
      endLearnTime = beginTime + (endTime - beginTime) / 2.0;
    }
    double beginTestTime = timeTesting;
    if (timeTesting < 0.0)
    {
      beginTestTime = beginTime + (endTime - beginTime) / 2.0;
    }
    double endTestTime = endTime - 10.0;
    // Do learning
    bool isSuccess = modelWithMocap.regressionsLearn(beginLearnTime, endLearnTime);
    if (!isSuccess)
    {
      std::cout << "WARNING: LEARNING FAILED from " << beginLearnTime << " to " << endLearnTime << std::endl;
    }

    // Saving Regressions
    modelWithMocap.regressionsSave("/tmp/");
    // Displaying regression prediction error
    if (!isQuiet)
    {
      std::cout << "Regressions MSE:" << std::endl;
      modelWithMocap.regressionsPrintMSE(beginTestTime, endTestTime);
    }
  }

  // Load data into others models
  modelWithMocap.regressionsLoad("/tmp/");
  modelNoMocap.regressionsLoad("/tmp/");
  modelNoSensor.regressionsLoad("/tmp/");

  // Propagate regressions
  modelNoMocap.propagateRegressions();
  modelNoSensor.propagateRegressions();

  // Fix odometry integration too late
  double t1 =
      std::max(modelNoMocap.series("delta_mocap_x").timeMin(), modelWithMocap.series("integrated_mocap_x").timeMin());
  double t2 =
      std::max(modelNoSensor.series("delta_mocap_x").timeMin(), modelWithMocap.series("integrated_mocap_x").timeMin());
  modelNoMocap.series("integrated_mocap_x").append(t1, modelWithMocap.series("integrated_mocap_x").get(t1));
  modelNoMocap.series("integrated_mocap_y").append(t1, modelWithMocap.series("integrated_mocap_y").get(t1));
  modelNoMocap.series("integrated_mocap_theta").append(t1, modelWithMocap.series("integrated_mocap_theta").get(t1));
  modelNoSensor.series("integrated_mocap_x").append(t2, modelWithMocap.series("integrated_mocap_x").get(t2));
  modelNoSensor.series("integrated_mocap_y").append(t2, modelWithMocap.series("integrated_mocap_y").get(t2));
  modelNoSensor.series("integrated_mocap_theta").append(t2, modelWithMocap.series("integrated_mocap_theta").get(t2));
  // Propagate concepts
  modelNoMocap.propagateConcepts();
  modelNoSensor.propagateConcepts();
}

/**
 * Process mocal validity
 */
static void processRawDataMocapValidity(Leph::MatrixLabel& logs)
{
  auto func = [&logs](size_t index, int len) {
    int i = index;
    int count = 0;
    while (count < abs(len) && i < (int)logs.size() && i >= 0)
    {
      logs[i]("mocap:check") = 0.0;
      count++;
      if (len > 0)
      {
        i++;
      }
      else
      {
        i--;
      }
    }
  };

  size_t indexStart = -1;
  size_t indexStop = -1;
  bool isInvalid = false;
  for (size_t i = 0; i < logs.size(); i++)
  {
    logs[i].append("mocap:check", 1.0);
  }
  for (size_t i = 0; i < logs.size(); i++)
  {
    if (logs[i]("mocap:is_valid") < 0.5)
    {
      if (isInvalid)
      {
      }
      else
      {
        indexStart = i;
      }
      isInvalid = true;
    }
    else
    {
      if (isInvalid)
      {
        indexStop = i;
        if (indexStop - indexStart > 20)
        {
          func(indexStart, 2 * (indexStop - indexStart));
          func(indexStop, 2 * (indexStop - indexStart));
          func(indexStart, 2 * (indexStart - indexStop));
        }
      }
      else
      {
      }
      isInvalid = false;
    }
  }
}

/**
 * Retrieve and return from given data logs
 * sequence of mocap valid
 */
std::vector<std::pair<size_t, size_t>> processCutSequences(const Leph::MatrixLabel& logs)
{
  std::vector<std::pair<size_t, size_t>> result;

  size_t beginIndex = 0;
  size_t endIndex = 0;
  bool state = false;
  for (size_t i = 0; i < logs.size(); i++)
  {
    if (logs[i]("mocap:check") > 0.5)
    {
      if (state == false)
      {
        beginIndex = i;
      }
      else if (i - beginIndex > 50 * 20)
      {
        endIndex = i;
        result.push_back({ beginIndex, endIndex });
        beginIndex = i;
      }
      state = true;
    }
    else
    {
      if (state == true)
      {
        endIndex = i;
        if (endIndex - beginIndex > 50 * 10)
        {
          result.push_back({ beginIndex, endIndex });
        }
      }
      state = false;
    }
  }
  if (state == true)
  {
    endIndex = logs.size() - 1;
    result.push_back({ beginIndex, endIndex });
  }

  return result;
}

/**
 * Plot given models
 */
static void plotModels(const Leph::ModelSeries& modelWithMocap, const Leph::ModelSeries& modelNoMocap,
                       const Leph::ModelSeries& modelNoSensor, bool onlyOdometry = false)
{
  Leph::Plot plot;

  if (!onlyOdometry)
  {
    plot.add(modelWithMocap.series("mocap_is_valid"));
    plot.add(modelWithMocap.series("mocap_x"));
    plot.add(modelWithMocap.series("mocap_y"));
    plot.add(modelWithMocap.series("mocap_theta"));
    plot.add(modelWithMocap.series("delta_mocap_x"));
    plot.add(modelWithMocap.series("walk_step"));
    plot.add(modelWithMocap.series("walk_lateral"));
    plot.add(modelWithMocap.series("walk_turn"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add(modelWithMocap.series("sensor_roll"));
    plot.add(modelWithMocap.series("sensor_pitch"));
    plot.add(modelWithMocap.series("mocap_x"));
    plot.add(modelWithMocap.series("mocap_y"));
    plot.add(modelWithMocap.series("pos_is_support_foot_left"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("x_mocap_delta", modelWithMocap.series("delta_mocap_x"));
    plot.add("x_model_delta", modelNoMocap.series("pos_delta_head_x"));
    plot.add("x_learn_delta", modelNoMocap.series("delta_mocap_x"));
    plot.add("x_walk_delta", modelNoSensor.series("delta_mocap_x"));
    plot.add("x_order_delta", modelNoSensor.series("goal_delta_head_x"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("y_mocap_delta", modelWithMocap.series("delta_mocap_y"));
    plot.add("y_model_delta", modelNoMocap.series("pos_delta_head_y"));
    plot.add("y_learn_delta", modelNoMocap.series("delta_mocap_y"));
    plot.add("y_walk_delta", modelNoSensor.series("delta_mocap_y"));
    plot.add("y_order_delta", modelNoSensor.series("goal_delta_head_y"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("theta_mocap_delta", modelWithMocap.series("delta_mocap_theta"));
    plot.add("theta_model_delta", modelNoMocap.series("pos_delta_head_theta"));
    plot.add("theta_learn_delta", modelNoMocap.series("delta_mocap_theta"));
    plot.add("theta_walk_delta", modelNoSensor.series("delta_mocap_theta"));
    plot.add("theta_order_delta", modelNoSensor.series("goal_delta_head_theta"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("mocap_x", modelWithMocap.series("integrated_mocap_x"));
    plot.add("model_x", modelNoMocap.series("integrated_head_x"));
    plot.add("learn_x", modelNoMocap.series("integrated_mocap_x"));
    plot.add("walk_x", modelNoSensor.series("integrated_mocap_x"));
    plot.add("order_x", modelNoSensor.series("integrated_walk_x"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("mocap_y", modelWithMocap.series("integrated_mocap_y"));
    plot.add("model_y", modelNoMocap.series("integrated_head_y"));
    plot.add("learn_y", modelNoMocap.series("integrated_mocap_y"));
    plot.add("walk_y", modelNoSensor.series("integrated_mocap_y"));
    plot.add("order_y", modelNoSensor.series("integrated_walk_y"));
    plot.plot("time", "all").render();

    plot.clear();
    plot.add("mocap_theta", modelWithMocap.series("integrated_mocap_theta"));
    plot.add("model_theta", modelNoMocap.series("integrated_head_theta"));
    plot.add("learn_theta", modelNoMocap.series("integrated_mocap_theta"));
    plot.add("walk_theta", modelNoSensor.series("integrated_mocap_theta"));
    plot.add("order_theta", modelNoSensor.series("integrated_walk_theta"));
    plot.plot("time", "all").render();
    plot.clear();
  }

  Leph::plotPhase(plot, modelWithMocap, "mocap_x", "integrated_mocap_x", "mocap_y", "integrated_mocap_y");
  Leph::plotPhase(plot, modelNoMocap, "learn_x", "integrated_mocap_x", "learn_y", "integrated_mocap_y");
  Leph::plotPhase(plot, modelNoMocap, "model_x", "integrated_head_x", "model_y", "integrated_head_y");
  Leph::plotPhase(plot, modelNoSensor, "walk_x", "integrated_mocap_x", "walk_y", "integrated_mocap_y");
  Leph::plotPhase(plot, modelNoSensor, "order_x", "integrated_walk_x", "order_y", "integrated_walk_y");
  plot.plot("mocap_x", "mocap_y")
      .plot("learn_x", "learn_y")
      .plot("model_x", "model_y")
      .plot("walk_x", "walk_y")
      .plot("order_x", "order_y")
      .render();
  plot.clear();
}

/**
 * Optimize all regressions meta parameters from given
 * data logs and save then in /tmp folder
 */
static void computeAndFindMetaParameters(const Leph::MatrixLabel& logs, bool invMocap, int maxIteration, int retry,
                                         bool doFullLearning = false)
{
  // Initialize ModelSeries
  Leph::ModelSeries modelWithMocap;
  Leph::initModelSeries(modelWithMocap, true, true, true);
  // Load data into TimeSeries
  for (size_t i = 0; i < logs.size(); i++)
  {
    double time = logs[i]("time:timestamp") / 1000.0;
    Leph::appendModelSeries(modelWithMocap, time, logs[i], invMocap);
  }
  // Compute all values throught Concepts graph
  modelWithMocap.propagateConcepts();

  double beginTime = modelWithMocap.series("mocap_x").timeMin();
  double endTime = modelWithMocap.series("mocap_x").timeMax();
  double beginLearnTime = beginTime + 5.0;
  double endLearnTime = beginTime + (endTime - beginTime) / 2.0;
  double beginTestTime = beginTime + (endTime - beginTime) / 2.0;
  double endTestTime = endTime - 10.0;
  // Optimize and save meta parameters
  modelWithMocap.regressionsOptimizeParameters(beginLearnTime, endLearnTime, beginTestTime, endTestTime, maxIteration,
                                               true, retry, "/tmp/");
  // Learn and log part
  if (doFullLearning)
  {
    modelWithMocap.regressionsLearn(endLearnTime, endTestTime);
  }
  // Saving Regressions
  modelWithMocap.regressionsSave("/tmp/");
}

/**
 * Load given data file name into given
 * container and update min/max time
 */
static void loadDataFiles(const std::vector<std::string>& fileLogsLearn, std::vector<Leph::MatrixLabel>& dataLogsLearn,
                          double& dataTimeLearnMin, double& dataTimeLearnMax)
{
  // Load data from model logs
  std::cout << "Loading data" << std::endl;
  dataTimeLearnMin = -1.0;
  dataTimeLearnMax = -1.0;
  dataLogsLearn.clear();
  for (size_t i = 0; i < fileLogsLearn.size(); i++)
  {
    // Data loading
    dataLogsLearn.push_back(Leph::MatrixLabel());
    dataLogsLearn.back().load(fileLogsLearn[i]);
    // Retrieve data min and max time
    size_t size = dataLogsLearn.back().size();
    size_t dim = dataLogsLearn.back().dimension();
    double tMin = dataLogsLearn.back()[0]("time:timestamp") / 1000.0;
    double tMax = dataLogsLearn.back()[size - 1]("time:timestamp") / 1000.0;
    if (dataTimeLearnMin < 0.0 || dataTimeLearnMin < tMin)
    {
      dataTimeLearnMin = tMin;
    }
    if (dataTimeLearnMax < 0.0 || dataTimeLearnMax > tMax)
    {
      dataTimeLearnMax = tMax;
    }
    // Print data informations
    std::cout << "Loaded Learn " << fileLogsLearn[i] << ": " << size << " points with " << dim
              << " entries from t=" << tMin << "..." << tMax << std::endl;
  }
  // Compute data middle and length time
  std::cout << "dataTimeLearnMin=" << dataTimeLearnMin << " dataTimeLearnMax=" << dataTimeLearnMax << std::endl;

  // Do some data post processing
  std::cout << "Data post processing" << std::endl;
  for (size_t j = 0; j < dataLogsLearn.size(); j++)
  {
    // Analyse Motion Capture data validity
    processRawDataMocapValidity(dataLogsLearn[j]);
  }
}

/**
 * Plot 1
 * Show data prediction error convergence
 */
static void makePlotConvergence()
{
  // Learn logs filename container
  std::vector<std::string> fileLogsLearn = {
    // Artificial grass open loop
    //"../../These/Data/model_2015-09-07-18-36-45.log", //Too few points
    "../../These/Data/model_2015-09-07-18-56-56.log",
    "../../These/Data/model_2015-09-07-19-08-06.log",
    "../../These/Data/model_2015-09-07-19-22-53.log",
    "../../These/Data/model_2015-09-07-19-31-25.log",
  };

  // Load data into MatrixLabel and post proccess it
  double dataTimeLearnMin;
  double dataTimeLearnMax;
  std::vector<Leph::MatrixLabel> dataLogsLearn;
  loadDataFiles(fileLogsLearn, dataLogsLearn, dataTimeLearnMin, dataTimeLearnMax);
  double dataTimeLearnLength = dataTimeLearnMax - dataTimeLearnMin;
  double dataTimeLearnMiddle = 0.75 * dataTimeLearnMax + 0.25 * dataTimeLearnMin;

  // Optimize model
  computeAndFindMetaParameters(dataLogsLearn[0], true, 100, 1);

  // Generate the data statistics
  Leph::Plot plotData;
  for (double time = dataTimeLearnMin + 5.0; time <= dataTimeLearnMiddle; time += dataTimeLearnLength / 20.0)
  {
    // Init statitics container
    std::vector<Gaussian> modelX;
    std::vector<Gaussian> modelY;
    std::vector<Gaussian> modelTheta;
    std::vector<Gaussian> modelLearnX;
    std::vector<Gaussian> modelLearnY;
    std::vector<Gaussian> modelLearnTheta;
    std::vector<Gaussian> walkX;
    std::vector<Gaussian> walkY;
    std::vector<Gaussian> walkTheta;
    std::vector<Gaussian> walkLearnX;
    std::vector<Gaussian> walkLearnY;
    std::vector<Gaussian> walkLearnTheta;
    for (size_t i = 0; i < dataLogsLearn.size(); i++)
    {
      // Init models
      Leph::ModelSeries modelWithMocap;
      Leph::ModelSeries modelNoMocap;
      Leph::ModelSeries modelNoSensor;
      setUpModels(dataLogsLearn[i], true, -1, -1,  // Sub sequence
                  modelWithMocap, modelNoMocap, modelNoSensor,
                  false,                // doPreLoad
                  true,                 // doLearning
                  time,                 // timeLearning
                  dataTimeLearnMiddle,  // timeTesting
                  true);                // isQuiet

      // Lambda computing error statistics on given
      // TimeSeries name
      auto func = [&modelWithMocap, &dataTimeLearnMiddle](Leph::ModelSeries& model, const std::string& name1,
                                                          const std::string& name2) -> Gaussian {
        // Time interval
        double beginTime = dataTimeLearnMiddle;
        double endTime = modelWithMocap.series("mocap_x").timeMax() - 10.0;
        // Compute error
        double sumError;
        double sumSquaredError;
        int count;
        Leph::seriesCompare(modelWithMocap.series(name1), model.series(name2), beginTime, endTime, sumError,
                            sumSquaredError, count);
        // Return stats
        Gaussian stats;
        stats.mean = sumError / (double)count;
        stats.var = sumSquaredError / (double)count - pow(sumError / (double)count, 2);
        stats.count = count;
        return stats;
      };

      // Computing statistics
      std::cout << "Generated time=" << time << " log=" << i << std::endl;
      modelX.push_back(func(modelNoMocap, "delta_mocap_x", "pos_delta_head_x"));
      modelY.push_back(func(modelNoMocap, "delta_mocap_y", "pos_delta_head_y"));
      modelTheta.push_back(func(modelNoMocap, "delta_mocap_theta", "pos_delta_head_theta"));
      modelLearnX.push_back(func(modelNoMocap, "delta_mocap_x", "delta_mocap_x"));
      modelLearnY.push_back(func(modelNoMocap, "delta_mocap_y", "delta_mocap_y"));
      modelLearnTheta.push_back(func(modelNoMocap, "delta_mocap_theta", "delta_mocap_theta"));
      walkX.push_back(func(modelNoSensor, "delta_mocap_x", "goal_delta_head_x"));
      walkY.push_back(func(modelNoSensor, "delta_mocap_y", "goal_delta_head_y"));
      walkTheta.push_back(func(modelNoSensor, "delta_mocap_theta", "goal_delta_head_theta"));
      walkLearnX.push_back(func(modelNoSensor, "delta_mocap_x", "delta_mocap_x"));
      walkLearnY.push_back(func(modelNoSensor, "delta_mocap_y", "delta_mocap_y"));
      walkLearnTheta.push_back(func(modelNoSensor, "delta_mocap_theta", "delta_mocap_theta"));
    }
    Gaussian modelXGaussian = mergeGaussian(modelX);
    Gaussian modelYGaussian = mergeGaussian(modelY);
    Gaussian modelThetaGaussian = mergeGaussian(modelTheta);
    Gaussian modelLearnXGaussian = mergeGaussian(modelLearnX);
    Gaussian modelLearnYGaussian = mergeGaussian(modelLearnY);
    Gaussian modelLearnThetaGaussian = mergeGaussian(modelLearnTheta);
    Gaussian walkXGaussian = mergeGaussian(walkX);
    Gaussian walkYGaussian = mergeGaussian(walkY);
    Gaussian walkThetaGaussian = mergeGaussian(walkTheta);
    Gaussian walkLearnXGaussian = mergeGaussian(walkLearnX);
    Gaussian walkLearnYGaussian = mergeGaussian(walkLearnY);
    Gaussian walkLearnThetaGaussian = mergeGaussian(walkLearnTheta);
    plotData.add(Leph::VectorLabel(
        "time", time, "model_x", modelXGaussian.mean, "model_x_error", confidenceBounds(modelXGaussian), "model_y",
        modelYGaussian.mean, "model_y_error", confidenceBounds(modelYGaussian), "model_theta", modelThetaGaussian.mean,
        "model_theta_error", confidenceBounds(modelThetaGaussian), "model_learn_x", modelLearnXGaussian.mean,
        "model_learn_x_error", confidenceBounds(modelLearnXGaussian), "model_learn_y", modelLearnYGaussian.mean,
        "model_learn_y_error", confidenceBounds(modelLearnYGaussian), "model_learn_theta", modelLearnThetaGaussian.mean,
        "model_learn_theta_error", confidenceBounds(modelLearnThetaGaussian), "walk_x", walkXGaussian.mean,
        "walk_x_error", confidenceBounds(walkXGaussian), "walk_y", walkYGaussian.mean, "walk_y_error",
        confidenceBounds(walkYGaussian), "walk_theta", walkThetaGaussian.mean, "walk_theta_error",
        confidenceBounds(walkThetaGaussian), "walk_learn_x", walkLearnXGaussian.mean, "walk_learn_x_error",
        confidenceBounds(walkLearnXGaussian), "walk_learn_y", walkLearnYGaussian.mean, "walk_learn_y_error",
        confidenceBounds(walkLearnYGaussian), "walk_learn_theta", walkLearnThetaGaussian.mean, "walk_learn_theta_error",
        confidenceBounds(walkLearnThetaGaussian)));
  }
  plotData.plot("time", "model_x", Leph::Plot::ErrorsLines, "model_x_error")
      .plot("time", "model_learn_x", Leph::Plot::ErrorsLines, "model_learn_x_error")
      .plot("time", "walk_x", Leph::Plot::ErrorsLines, "walk_x_error")
      .plot("time", "walk_learn_x", Leph::Plot::ErrorsLines, "walk_learn_x_error")
      .render();
  plotData.plot("time", "model_y", Leph::Plot::ErrorsLines, "model_y_error")
      .plot("time", "model_learn_y", Leph::Plot::ErrorsLines, "model_learn_y_error")
      .plot("time", "walk_y", Leph::Plot::ErrorsLines, "walk_y_error")
      .plot("time", "walk_learn_y", Leph::Plot::ErrorsLines, "walk_learn_y_error")
      .render();
  plotData.plot("time", "model_theta", Leph::Plot::ErrorsLines, "model_theta_error")
      .plot("time", "model_learn_theta", Leph::Plot::ErrorsLines, "model_learn_theta_error")
      .plot("time", "walk_theta", Leph::Plot::ErrorsLines, "walk_theta_error")
      .plot("time", "walk_learn_theta", Leph::Plot::ErrorsLines, "walk_learn_theta_error")
      .render();
  plotData.clear();
}

/**
 * Plot 2
 * Validate the quality of different methods
 * of odometry computations
 */
static void makePlotOdometry()
{
  // Learn logs filename container
  std::vector<std::string> fileLogsLearn = {
    // Grass open loop
    "../../These/Data/model_2015-09-07-19-22-53.log", "../../These/Data/model_2015-09-07-18-36-45.log",
    "../../These/Data/model_2015-09-07-18-56-56.log", "../../These/Data/model_2015-09-07-19-08-06.log",
    "../../These/Data/model_2015-09-07-19-31-25.log",
  };

  // Load data into MatrixLabel and post proccess it
  double dataTimeLearnMin;
  double dataTimeLearnMax;
  std::vector<Leph::MatrixLabel> dataLogsLearn;
  loadDataFiles(fileLogsLearn, dataLogsLearn, dataTimeLearnMin, dataTimeLearnMax);

  // Optimize model
  computeAndFindMetaParameters(dataLogsLearn[0], true, 100, 1, true);
  /*
  for (size_t i=1;i<dataLogsLearn.size();i++) {
      //Init models
      Leph::ModelSeries modelWithMocap;
      Leph::ModelSeries modelNoMocap;
      Leph::ModelSeries modelNoSensor;
      std::cout << "Learning " << i << " from "
          << dataLogsLearn[i][dataLogsLearn[i].size()/2]("time:timestamp")/1000.0 << std::endl;
      setUpModels(dataLogsLearn[i], true,
          -1, -1,
          modelWithMocap,
          modelNoMocap,
          modelNoSensor,
          true, //doPreLoad
          true, //doLearning
          dataLogsLearn[i][dataLogsLearn[i].size()/2]("time:timestamp")/1000.0, //timeLearning
          dataLogsLearn[i][dataLogsLearn[i].size()/2]("time:timestamp")/1000.0, //timeTesting
          false); //isQuiet
  }
  */

  Leph::Plot plotData;
  // Odometry statistics
  std::vector<std::vector<Gaussian>> statsDistModel;
  std::vector<std::vector<Gaussian>> statsDistLearn;
  std::vector<std::vector<Gaussian>> statsDistWalk;
  std::vector<std::vector<Gaussian>> statsDistOrder;
  std::vector<std::vector<Gaussian>> statsAngleModel;
  std::vector<std::vector<Gaussian>> statsAngleLearn;
  std::vector<std::vector<Gaussian>> statsAngleWalk;
  std::vector<std::vector<Gaussian>> statsAngleOrder;
  for (size_t i = 1; i < dataLogsLearn.size(); i++)
  {
    // Optimize Model
    std::cout << "Learning log " << i << std::endl;
    // Cutting learn data into tests sequences
    std::vector<std::pair<size_t, size_t>> seqs = processCutSequences(dataLogsLearn[i]);
    // For all test sequences
    for (size_t j = 0; j < seqs.size(); j++)
    {
      size_t len = seqs[j].second - seqs[j].first;
      std::cout << "Log=" << i << " Sequence length=" << len / 50.0 << " Start=" << seqs[j].first << std::endl;
      // Load model
      Leph::ModelSeries tmpModelWithMocap;
      Leph::ModelSeries tmpModelNoMocap;
      Leph::ModelSeries tmpModelNoSensor;
      setUpModels(dataLogsLearn[i], true, seqs[j].first, seqs[j].second, tmpModelWithMocap, tmpModelNoMocap,
                  tmpModelNoSensor,
                  false,  // doPreLoad
                  false,  // doLearning
                  0,      // timeLearning
                  0,      // timeTesting
                  true);  // isQuiet
      // Compute odometry cartesian errors
      double timeMin = tmpModelNoMocap.series("integrated_mocap_x").timeMin();
      double timeMax = tmpModelNoMocap.series("integrated_mocap_x").timeMax();
      size_t index = 0;
      while (timeMin + index * 1.0 + 1.0 < timeMax)
      {
        double time = timeMin + index * 1.0 + 1.0;
        while (statsDistModel.size() < index + 1)
          statsDistModel.push_back(std::vector<Gaussian>());
        while (statsDistLearn.size() < index + 1)
          statsDistLearn.push_back(std::vector<Gaussian>());
        while (statsDistWalk.size() < index + 1)
          statsDistWalk.push_back(std::vector<Gaussian>());
        while (statsDistOrder.size() < index + 1)
          statsDistOrder.push_back(std::vector<Gaussian>());
        while (statsAngleModel.size() < index + 1)
          statsAngleModel.push_back(std::vector<Gaussian>());
        while (statsAngleLearn.size() < index + 1)
          statsAngleLearn.push_back(std::vector<Gaussian>());
        while (statsAngleWalk.size() < index + 1)
          statsAngleWalk.push_back(std::vector<Gaussian>());
        while (statsAngleOrder.size() < index + 1)
          statsAngleOrder.push_back(std::vector<Gaussian>());
        double mocapX = tmpModelWithMocap.series("integrated_mocap_x").get(time);
        double mocapY = tmpModelWithMocap.series("integrated_mocap_y").get(time);
        double modelX = tmpModelNoMocap.series("integrated_head_x").get(time);
        double modelY = tmpModelNoMocap.series("integrated_head_y").get(time);
        double learnX = tmpModelNoMocap.series("integrated_mocap_x").get(time);
        double learnY = tmpModelNoMocap.series("integrated_mocap_y").get(time);
        double walkX = tmpModelNoSensor.series("integrated_mocap_x").get(time);
        double walkY = tmpModelNoSensor.series("integrated_mocap_y").get(time);
        double orderX = tmpModelNoSensor.series("integrated_walk_x").get(time);
        double orderY = tmpModelNoSensor.series("integrated_walk_y").get(time);
        double distModel = sqrt(pow(mocapX - modelX, 2) + pow(mocapY - modelY, 2));
        double distLearn = sqrt(pow(mocapX - learnX, 2) + pow(mocapY - learnY, 2));
        double distWalk = sqrt(pow(mocapX - walkX, 2) + pow(mocapY - walkY, 2));
        double distOrder = sqrt(pow(mocapX - orderX, 2) + pow(mocapY - orderY, 2));
        double mocapAngle = tmpModelWithMocap.series("integrated_mocap_theta").get(time);
        double modelAngle = tmpModelNoMocap.series("integrated_head_theta").get(time);
        double learnAngle = tmpModelNoMocap.series("integrated_mocap_theta").get(time);
        double walkAngle = tmpModelNoSensor.series("integrated_mocap_theta").get(time);
        double orderAngle = tmpModelNoSensor.series("integrated_walk_theta").get(time);
        double distModelAngle = fabs(Leph::AngleDistance(mocapAngle, modelAngle));
        double distLearnAngle = fabs(Leph::AngleDistance(mocapAngle, learnAngle));
        double distWalkAngle = fabs(Leph::AngleDistance(mocapAngle, walkAngle));
        double distOrderAngle = fabs(Leph::AngleDistance(mocapAngle, orderAngle));
        statsDistModel[index].push_back({ distModel, 0.0, 1.0 });
        statsDistLearn[index].push_back({ distLearn, 0.0, 1.0 });
        statsDistWalk[index].push_back({ distWalk, 0.0, 1.0 });
        statsDistOrder[index].push_back({ distOrder, 0.0, 1.0 });
        statsAngleModel[index].push_back({ distModelAngle, 0.0, 1.0 });
        statsAngleLearn[index].push_back({ distLearnAngle, 0.0, 1.0 });
        statsAngleWalk[index].push_back({ distWalkAngle, 0.0, 1.0 });
        statsAngleOrder[index].push_back({ distOrderAngle, 0.0, 1.0 });
        index++;
      }
      // plotModels(tmpModelWithMocap, tmpModelNoMocap, tmpModelNoSensor, true);
    }
  }
  // Merge computed statistics
  for (size_t j = 0; j < statsDistModel.size(); j++)
  {
    Gaussian mergedDistModel = mergeGaussian(statsDistModel[j]);
    Gaussian mergedDistLearn = mergeGaussian(statsDistLearn[j]);
    Gaussian mergedDistWalk = mergeGaussian(statsDistWalk[j]);
    Gaussian mergedDistOrder = mergeGaussian(statsDistOrder[j]);
    Gaussian mergedAngleModel = mergeGaussian(statsAngleModel[j]);
    Gaussian mergedAngleLearn = mergeGaussian(statsAngleLearn[j]);
    Gaussian mergedAngleWalk = mergeGaussian(statsAngleWalk[j]);
    Gaussian mergedAngleOrder = mergeGaussian(statsAngleOrder[j]);
    std::cout << "time=" << j * 1.0 + 1.0 << " count=" << statsDistModel[j].size() << std::endl;
    if (statsDistModel[j].size() < 5)
    {
      std::cout << "WARNING low statistics" << std::endl;
    }
    plotData.add(
        Leph::VectorLabel("time", j * 1.0 + 1.0, "model_dist", mergedDistModel.mean, "model_dist_error",
                          confidenceBounds(mergedDistModel), "learn_dist", mergedDistLearn.mean, "learn_dist_error",
                          confidenceBounds(mergedDistLearn), "walk_dist", mergedDistWalk.mean, "walk_dist_error",
                          confidenceBounds(mergedDistWalk), "order_dist", mergedDistOrder.mean, "order_dist_error",
                          confidenceBounds(mergedDistOrder), "model_angle", mergedAngleModel.mean, "model_angle_error",
                          confidenceBounds(mergedAngleModel), "learn_angle", mergedAngleLearn.mean, "learn_angle_error",
                          confidenceBounds(mergedAngleLearn), "walk_angle", mergedAngleWalk.mean, "walk_angle_error",
                          confidenceBounds(mergedAngleWalk), "order_angle", mergedAngleOrder.mean, "order_angle_error",
                          confidenceBounds(mergedAngleOrder)));
  }
  // Plot
  plotData.plot("time", "model_dist", Leph::Plot::ErrorsLines, "model_dist_error")
      .plot("time", "learn_dist", Leph::Plot::ErrorsLines, "learn_dist_error")
      .plot("time", "walk_dist", Leph::Plot::ErrorsLines, "walk_dist_error")
      .plot("time", "order_dist", Leph::Plot::ErrorsLines, "order_dist_error")
      .render();
  plotData.plot("time", "model_angle", Leph::Plot::ErrorsLines, "model_angle_error")
      .plot("time", "learn_angle", Leph::Plot::ErrorsLines, "learn_angle_error")
      .plot("time", "walk_angle", Leph::Plot::ErrorsLines, "walk_angle_error")
      .plot("time", "order_angle", Leph::Plot::ErrorsLines, "order_angle_error")
      .render();
  plotData.clear();
}

/**
 * Plot 3
 * Show typical 2d cartesian odometry trajectories
 */
static void makePlotTrajectory()
{
  // Learn logs filename container
  std::vector<std::string> fileLogsLearn = {
    // Grass open loop
    "../../These/Data/model_2015-09-07-19-22-53.log", "../../These/Data/model_2015-09-07-18-36-45.log",
    "../../These/Data/model_2015-09-07-18-56-56.log", "../../These/Data/model_2015-09-07-19-08-06.log",
    "../../These/Data/model_2015-09-07-19-31-25.log",
  };

  // Load data into MatrixLabel and post proccess it
  double dataTimeLearnMin;
  double dataTimeLearnMax;
  std::vector<Leph::MatrixLabel> dataLogsLearn;
  loadDataFiles(fileLogsLearn, dataLogsLearn, dataTimeLearnMin, dataTimeLearnMax);

  // Optimize model
  computeAndFindMetaParameters(dataLogsLearn[0], true, 100, 1, true);

  // Display some of trajectory
  for (size_t i = 1; i < dataLogsLearn.size(); i++)
  {
    // Cutting learn data into tests sequences
    std::vector<std::pair<size_t, size_t>> seqs = processCutSequences(dataLogsLearn[i]);
    // For all test sequences
    for (size_t j = 0; j < seqs.size(); j++)
    {
      if ((i == 1 && j == 2) || (i == 1 && j == 7) || (i == 1 && j == 8) || (i == 1 && j == 10) || (i == 2 && j == 1) ||
          (i == 4 && j == 6))
      {
        size_t len = seqs[j].second - seqs[j].first;
        std::cout << "Log=" << i << " Sequence length=" << len / 50.0 << " Start=" << seqs[j].first << std::endl;
        // Load model
        Leph::ModelSeries tmpModelWithMocap;
        Leph::ModelSeries tmpModelNoMocap;
        Leph::ModelSeries tmpModelNoSensor;
        setUpModels(dataLogsLearn[i], true, seqs[j].first, seqs[j].second, tmpModelWithMocap, tmpModelNoMocap,
                    tmpModelNoSensor,
                    false,  // doPreLoad
                    false,  // doLearning
                    0,      // timeLearning
                    0,      // timeTesting
                    true);  // isQuiet
        plotModels(tmpModelWithMocap, tmpModelNoMocap, tmpModelNoSensor, true);
      }
    }
  }
}

/**
 * Plot 4
 * Show difference between closed/open
 * loop and carpet/grass
 */
static void makePlotCompare()
{
  // Learn logs filename container
  std::vector<std::string> fileLogsGrassOpen = {
    // Grass open loop
    "../../These/Data/model_2015-09-07-19-22-53.log", "../../These/Data/model_2015-09-07-18-36-45.log",
    "../../These/Data/model_2015-09-07-18-56-56.log", "../../These/Data/model_2015-09-07-19-08-06.log",
    "../../These/Data/model_2015-09-07-19-31-25.log",
  };
  std::vector<std::string> fileLogsGrassClose = {
    // Grass close loop
    "../../These/Data/model_2015-09-08-13-14-43.log", "../../These/Data/model_2015-09-08-12-57-14.log",
    "../../These/Data/model_2015-09-08-12-34-30.log", "../../These/Data/model_2015-09-08-12-43-20.log",
    "../../These/Data/model_2015-09-08-13-00-55.log", "../../These/Data/model_2015-09-08-13-08-03.log",
  };
  std::vector<std::string> fileLogsCarpetOpen = {
    // Carpet open loop
    "../../These/Data/model_2015-09-07-23-13-14.log", "../../These/Data/model_2015-09-07-22-50-54.log",
    "../../These/Data/model_2015-09-07-23-02-59.log", "../../These/Data/model_2015-09-07-23-29-48.log",
    "../../These/Data/model_2015-09-07-23-44-20.log",
  };
  std::vector<std::string> fileLogsCarpetClose = {
    // Carpet close loop
    "../../These/Data/model_2015-09-08-14-59-11.log",
    "../../These/Data/model_2015-09-08-15-07-29.log",
    "../../These/Data/model_2015-09-08-15-14-19.log",
    "../../These/Data/model_2015-09-08-15-27-13.log",
  };

  // Load data into MatrixLabel and post proccess it
  double dataTimeMinGrassOpen;
  double dataTimeMaxGrassOpen;
  std::vector<Leph::MatrixLabel> dataLogsGrassOpen;
  loadDataFiles(fileLogsGrassOpen, dataLogsGrassOpen, dataTimeMinGrassOpen, dataTimeMaxGrassOpen);
  double dataTimeMinGrassClose;
  double dataTimeMaxGrassClose;
  std::vector<Leph::MatrixLabel> dataLogsGrassClose;
  loadDataFiles(fileLogsGrassClose, dataLogsGrassClose, dataTimeMinGrassClose, dataTimeMaxGrassClose);
  double dataTimeMinCarpetOpen;
  double dataTimeMaxCarpetOpen;
  std::vector<Leph::MatrixLabel> dataLogsCarpetOpen;
  loadDataFiles(fileLogsCarpetOpen, dataLogsCarpetOpen, dataTimeMinCarpetOpen, dataTimeMaxCarpetOpen);
  double dataTimeMinCarpetClose;
  double dataTimeMaxCarpetClose;
  std::vector<Leph::MatrixLabel> dataLogsCarpetClose;
  loadDataFiles(fileLogsCarpetClose, dataLogsCarpetClose, dataTimeMinCarpetClose, dataTimeMaxCarpetClose);

  // Factorisation lamda
  // Compute and plot odometry errors statistics
  auto func = [](const std::vector<Leph::MatrixLabel>& dataLogs, bool invMocap) {
    // Optimize model
    computeAndFindMetaParameters(dataLogs[0], invMocap, 100, 1, true);
    Leph::Plot plotData;
    // Odometry statistics
    std::vector<std::vector<Gaussian>> statsDistModel;
    std::vector<std::vector<Gaussian>> statsDistLearn;
    std::vector<std::vector<Gaussian>> statsDistWalk;
    std::vector<std::vector<Gaussian>> statsDistOrder;
    std::vector<std::vector<Gaussian>> statsAngleModel;
    std::vector<std::vector<Gaussian>> statsAngleLearn;
    std::vector<std::vector<Gaussian>> statsAngleWalk;
    std::vector<std::vector<Gaussian>> statsAngleOrder;
    for (size_t i = 1; i < dataLogs.size(); i++)
    {
      // Optimize Model
      std::cout << "Learning log " << i << std::endl;
      // Cutting learn data into tests sequences
      std::vector<std::pair<size_t, size_t>> seqs = processCutSequences(dataLogs[i]);
      // For all test sequences
      for (size_t j = 0; j < seqs.size(); j++)
      {
        size_t len = seqs[j].second - seqs[j].first;
        std::cout << "Log=" << i << " Sequence length=" << len / 50.0 << " Start=" << seqs[j].first << std::endl;
        // Load model
        Leph::ModelSeries tmpModelWithMocap;
        Leph::ModelSeries tmpModelNoMocap;
        Leph::ModelSeries tmpModelNoSensor;
        setUpModels(dataLogs[i], invMocap, seqs[j].first, seqs[j].second, tmpModelWithMocap, tmpModelNoMocap,
                    tmpModelNoSensor,
                    false,  // doPreLoad
                    false,  // doLearning
                    0,      // timeLearning
                    0,      // timeTesting
                    true);  // isQuiet
        // Compute odometry cartesian errors
        double timeMin = tmpModelNoMocap.series("integrated_mocap_x").timeMin();
        double timeMax = tmpModelNoMocap.series("integrated_mocap_x").timeMax();
        size_t index = 0;
        while (timeMin + index * 1.0 + 1.0 < timeMax)
        {
          double time = timeMin + index * 1.0 + 1.0;
          while (statsDistModel.size() < index + 1)
            statsDistModel.push_back(std::vector<Gaussian>());
          while (statsDistLearn.size() < index + 1)
            statsDistLearn.push_back(std::vector<Gaussian>());
          while (statsDistWalk.size() < index + 1)
            statsDistWalk.push_back(std::vector<Gaussian>());
          while (statsDistOrder.size() < index + 1)
            statsDistOrder.push_back(std::vector<Gaussian>());
          while (statsAngleModel.size() < index + 1)
            statsAngleModel.push_back(std::vector<Gaussian>());
          while (statsAngleLearn.size() < index + 1)
            statsAngleLearn.push_back(std::vector<Gaussian>());
          while (statsAngleWalk.size() < index + 1)
            statsAngleWalk.push_back(std::vector<Gaussian>());
          while (statsAngleOrder.size() < index + 1)
            statsAngleOrder.push_back(std::vector<Gaussian>());
          double mocapX = tmpModelWithMocap.series("integrated_mocap_x").get(time);
          double mocapY = tmpModelWithMocap.series("integrated_mocap_y").get(time);
          double modelX = tmpModelNoMocap.series("integrated_head_x").get(time);
          double modelY = tmpModelNoMocap.series("integrated_head_y").get(time);
          double learnX = tmpModelNoMocap.series("integrated_mocap_x").get(time);
          double learnY = tmpModelNoMocap.series("integrated_mocap_y").get(time);
          double walkX = tmpModelNoSensor.series("integrated_mocap_x").get(time);
          double walkY = tmpModelNoSensor.series("integrated_mocap_y").get(time);
          double orderX = tmpModelNoSensor.series("integrated_walk_x").get(time);
          double orderY = tmpModelNoSensor.series("integrated_walk_y").get(time);
          double distModel = sqrt(pow(mocapX - modelX, 2) + pow(mocapY - modelY, 2));
          double distLearn = sqrt(pow(mocapX - learnX, 2) + pow(mocapY - learnY, 2));
          double distWalk = sqrt(pow(mocapX - walkX, 2) + pow(mocapY - walkY, 2));
          double distOrder = sqrt(pow(mocapX - orderX, 2) + pow(mocapY - orderY, 2));
          double mocapAngle = tmpModelWithMocap.series("integrated_mocap_theta").get(time);
          double modelAngle = tmpModelNoMocap.series("integrated_head_theta").get(time);
          double learnAngle = tmpModelNoMocap.series("integrated_mocap_theta").get(time);
          double walkAngle = tmpModelNoSensor.series("integrated_mocap_theta").get(time);
          double orderAngle = tmpModelNoSensor.series("integrated_walk_theta").get(time);
          double distModelAngle = fabs(Leph::AngleDistance(mocapAngle, modelAngle));
          double distLearnAngle = fabs(Leph::AngleDistance(mocapAngle, learnAngle));
          double distWalkAngle = fabs(Leph::AngleDistance(mocapAngle, walkAngle));
          double distOrderAngle = fabs(Leph::AngleDistance(mocapAngle, orderAngle));
          statsDistModel[index].push_back({ distModel, 0.0, 1.0 });
          statsDistLearn[index].push_back({ distLearn, 0.0, 1.0 });
          statsDistWalk[index].push_back({ distWalk, 0.0, 1.0 });
          statsDistOrder[index].push_back({ distOrder, 0.0, 1.0 });
          statsAngleModel[index].push_back({ distModelAngle, 0.0, 1.0 });
          statsAngleLearn[index].push_back({ distLearnAngle, 0.0, 1.0 });
          statsAngleWalk[index].push_back({ distWalkAngle, 0.0, 1.0 });
          statsAngleOrder[index].push_back({ distOrderAngle, 0.0, 1.0 });
          index++;
        }
      }
    }
    // Merge computed statistics
    for (size_t j = 0; j < statsDistModel.size(); j++)
    {
      Gaussian mergedDistModel = mergeGaussian(statsDistModel[j]);
      Gaussian mergedDistLearn = mergeGaussian(statsDistLearn[j]);
      Gaussian mergedDistWalk = mergeGaussian(statsDistWalk[j]);
      Gaussian mergedDistOrder = mergeGaussian(statsDistOrder[j]);
      Gaussian mergedAngleModel = mergeGaussian(statsAngleModel[j]);
      Gaussian mergedAngleLearn = mergeGaussian(statsAngleLearn[j]);
      Gaussian mergedAngleWalk = mergeGaussian(statsAngleWalk[j]);
      Gaussian mergedAngleOrder = mergeGaussian(statsAngleOrder[j]);
      std::cout << "time=" << j * 1.0 + 1.0 << " count=" << statsDistModel[j].size() << std::endl;
      if (statsDistModel[j].size() < 5)
      {
        std::cout << "WARNING low statistics" << std::endl;
      }
      plotData.add(Leph::VectorLabel(
          "time", j * 1.0 + 1.0, "model_dist", mergedDistModel.mean, "model_dist_error",
          confidenceBounds(mergedDistModel), "learn_dist", mergedDistLearn.mean, "learn_dist_error",
          confidenceBounds(mergedDistLearn), "walk_dist", mergedDistWalk.mean, "walk_dist_error",
          confidenceBounds(mergedDistWalk), "order_dist", mergedDistOrder.mean, "order_dist_error",
          confidenceBounds(mergedDistOrder), "model_angle", mergedAngleModel.mean, "model_angle_error",
          confidenceBounds(mergedAngleModel), "learn_angle", mergedAngleLearn.mean, "learn_angle_error",
          confidenceBounds(mergedAngleLearn), "walk_angle", mergedAngleWalk.mean, "walk_angle_error",
          confidenceBounds(mergedAngleWalk), "order_angle", mergedAngleOrder.mean, "order_angle_error",
          confidenceBounds(mergedAngleOrder)));
    }
    // Plot
    plotData.plot("time", "model_dist", Leph::Plot::ErrorsLines, "model_dist_error")
        .plot("time", "learn_dist", Leph::Plot::ErrorsLines, "learn_dist_error")
        .plot("time", "walk_dist", Leph::Plot::ErrorsLines, "walk_dist_error")
        .plot("time", "order_dist", Leph::Plot::ErrorsLines, "order_dist_error")
        .render();
    plotData.clear();
  };
  func(dataLogsGrassOpen, true);
  func(dataLogsGrassClose, false);
  func(dataLogsCarpetOpen, true);
  func(dataLogsCarpetClose, false);
}

int main()
{
  makePlotConvergence();
  makePlotOdometry();
  makePlotTrajectory();
  makePlotCompare();
  return 0;
}
