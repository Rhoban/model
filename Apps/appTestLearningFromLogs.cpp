#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include <lwpr_eigen.hpp>
#include "Utils/LWPRUtils.h"
#include "Plot/Plot.hpp"
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"

/**
 * First test of LWPR regression on
 * Sigmaban Logs. Learning prediction and
 * correlation of motors and sensors.
 */

/**
 * Input source
 */
typedef std::vector<std::pair<std::string, size_t>> Inputs;

/**
 * Load given logs data
 */
std::vector<Leph::MatrixLabel> loadLogs(const std::vector<std::string>& logFiles)
{
  std::vector<Leph::MatrixLabel> container;

  // Loading data
  for (size_t i = 0; i < logFiles.size(); i++)
  {
    Leph::MatrixLabel logs;
    logs.load(logFiles[i]);
    std::cout << "Loaded Log " << i << ": " << logs.size() << " points with " << logs.dimension() << " entries"
              << std::endl;
    container.push_back(logs);
  }

  return container;
}

/**
 * Compute and append model data
 * associated with captured logs
 */
void computeModelData(std::vector<Leph::MatrixLabel> logs)
{
  std::cout << "Computing model" << std::endl;
  for (size_t i = 0; i < logs.size(); i++)
  {
    // Initialize model
    Leph::HumanoidFixedModel modelOutputs(Leph::SigmabanModel);
    Leph::HumanoidFixedModel modelMotors(Leph::SigmabanModel);
    // Initialize VectorLabel DOF
    Leph::VectorLabel outputsDOF = logs[i][0].extract("output").rename("output", "");
    Leph::VectorLabel motorsDOF = logs[i][0].extract("motor").rename("motor", "");
    // Iterating over data
    for (size_t j = 0; j < logs[i].size(); j++)
    {
      // Retrieve DOF values
      outputsDOF.assignOp(logs[i][j], "output", "");
      motorsDOF.assignOp(logs[i][j], "motor", "");
      // Assigning
      modelOutputs.get().setDOF(outputsDOF, false);
      modelMotors.get().setDOF(motorsDOF, false);
      // Model update
      modelOutputs.updateBase();
      modelMotors.updateBase();
      modelMotors.setOrientation(
          Eigen::AngleAxisd(logs[i][j]("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
              Eigen::AngleAxisd(logs[i][j]("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
          false);
      // Compute model quantities
      logs[i][j].setOrAppend("model output:left_foot_x", modelOutputs.get().position("left_foot_tip", "origin").x());
      logs[i][j].setOrAppend("model output:left_foot_y", modelOutputs.get().position("left_foot_tip", "origin").y());
      logs[i][j].setOrAppend("model output:left_foot_z", modelOutputs.get().position("left_foot_tip", "origin").z());
      logs[i][j].setOrAppend("model output:right_foot_x", modelOutputs.get().position("right_foot_tip", "origin").x());
      logs[i][j].setOrAppend("model output:right_foot_y", modelOutputs.get().position("right_foot_tip", "origin").y());
      logs[i][j].setOrAppend("model output:right_foot_z", modelOutputs.get().position("right_foot_tip", "origin").z());
      logs[i][j].setOrAppend("model output:trunk_x", modelOutputs.get().position("trunk", "origin").x());
      logs[i][j].setOrAppend("model output:trunk_y", modelOutputs.get().position("trunk", "origin").y());
      logs[i][j].setOrAppend("model output:trunk_z", modelOutputs.get().position("trunk", "origin").z());
      logs[i][j].setOrAppend("model output:com_x", modelOutputs.get().centerOfMass("origin").x());
      logs[i][j].setOrAppend("model output:com_y", modelOutputs.get().centerOfMass("origin").y());
      logs[i][j].setOrAppend("model output:com_z", modelOutputs.get().centerOfMass("origin").z());
      logs[i][j].setOrAppend("model motor:left_foot_x", modelMotors.get().position("left_foot_tip", "origin").x());
      logs[i][j].setOrAppend("model motor:left_foot_y", modelMotors.get().position("left_foot_tip", "origin").y());
      logs[i][j].setOrAppend("model motor:left_foot_z", modelMotors.get().position("left_foot_tip", "origin").z());
      logs[i][j].setOrAppend("model motor:right_foot_x", modelMotors.get().position("right_foot_tip", "origin").x());
      logs[i][j].setOrAppend("model motor:right_foot_y", modelMotors.get().position("right_foot_tip", "origin").y());
      logs[i][j].setOrAppend("model motor:right_foot_z", modelMotors.get().position("right_foot_tip", "origin").z());
      logs[i][j].setOrAppend("model motor:trunk_x", modelMotors.get().position("trunk", "origin").x());
      logs[i][j].setOrAppend("model motor:trunk_y", modelMotors.get().position("trunk", "origin").y());
      logs[i][j].setOrAppend("model motor:trunk_z", modelMotors.get().position("trunk", "origin").z());
      logs[i][j].setOrAppend("model motor:com_x", modelMotors.get().centerOfMass("origin").x());
      logs[i][j].setOrAppend("model motor:com_y", modelMotors.get().centerOfMass("origin").y());
      logs[i][j].setOrAppend("model motor:com_z", modelMotors.get().centerOfMass("origin").z());
    }
  }
}

/**
 * Plot test data prediction
 */
void displayTest(LWPR_Object& model, const std::string& output, const Inputs& inputs,
                 const std::vector<Leph::MatrixLabel>& container)
{
  size_t inputDim = model.nIn();
  if (inputDim != inputs.size())
  {
    throw std::logic_error("Test invalid input");
  }

  Leph::Plot plot;
  Eigen::VectorXd x(inputDim);
  Eigen::VectorXd y(1);
  for (size_t j = 10; j < container.back().size(); j++)
  {
    for (size_t k = 0; k < inputs.size(); k++)
    {
      x(k) = container.back()[j - inputs[k].second](inputs[k].first);
    }
    y(0) = container.back()[j](output);
    // Predict future
    Eigen::VectorXd yp = model.predict(x);
    plot.add(Leph::VectorLabel("t", container.back()[j]("time:timestamp"), "y", y(0), "yp", yp(0)));
  }
  plot.plot("t", "all").render();
}

int main()
{
  std::vector<std::string> logFiles = {
    "/home/leph/Code/These/Data/model_2015-06-12-10-20-01.log"
    /*
    "../../These/Data/logs-2015-05-16/model_2015-05-16-18-51-58.log",
    "../../These/Data/logs-2015-05-16/model_2015-05-16-18-49-08.log",
    "../../These/Data/logs-2015-05-16/model_2015-05-16-18-49-57.log",
    "../../These/Data/logs-2015-05-16/model_2015-05-16-18-51-02.log"
    */
  };

  // Input and output definitions
  Inputs inputs = {
    //{"model motor:trunk_y", 8},
    { "model motor:trunk_y", 6 },
    //{"model motor:trunk_y", 4},
    //{"walk:trunkYOffset", 6},
    //{"walk:trunkXOffset", 6},
    //{"walk:trunkYOffset", 4},
    //{"walk:trunkXOffset", 4},
    //{"walk:trunkYOffset", 2},
    //{"walk:trunkXOffset", 2},
    { "walk:trunkYOffset", 2 },
    //{"walk:trunkXOffset", 1}
  };
  std::string output = "model motor:trunk_y";

  // Load data
  std::vector<Leph::MatrixLabel> container = loadLogs(logFiles);
  // Compute model related quantities
  computeModelData(container);

  // Do some plots
  container[0]
      .plot()
      .plot("index", "model output:left_foot_z")
      .plot("index", "model output:right_foot_z")
      .plot("index", "model motor:left_foot_z")
      .plot("index", "model motor:right_foot_z")
      .render();
  container[0]
      .plot()
      .plot("index", "model output:trunk_x")
      .plot("index", "model output:trunk_y")
      .plot("index", "model motor:trunk_x")
      .plot("index", "model motor:trunk_y")
      .render();
  container[0].plot().plot("walk:trunkYOffset", "walk:trunkXOffset", "model motor:trunk_y").render();

  // Generate Train and Test data vector
  std::vector<Eigen::VectorXd> trainInputs;
  std::vector<double> trainOutputs;
  std::vector<Eigen::VectorXd> testInputs;
  std::vector<double> testOutputs;
  for (size_t i = 0; i < (container.size() == 1 ? 1 : container.size() - 1); i++)
  {
    for (size_t j = 10; j < container[i].size(); j++)
    {
      Eigen::VectorXd x(inputs.size());
      for (size_t k = 0; k < inputs.size(); k++)
      {
        x(k) = container[i][j - inputs[k].second](inputs[k].first);
      }
      double y = container[i][j](output);
      // Append train data
      trainInputs.push_back(x);
      trainOutputs.push_back(y);
    }
  }
  for (size_t j = 10; j < container.back().size(); j++)
  {
    Eigen::VectorXd x(inputs.size());
    for (size_t k = 0; k < inputs.size(); k++)
    {
      x(k) = container.back()[j - inputs[k].second](inputs[k].first);
    }
    double y = container.back()[j](output);
    // Append test data
    testInputs.push_back(x);
    testOutputs.push_back(y);
  }

  // Init LWPR parameters
  Eigen::VectorXd metaParams = Leph::LWPRInitParameters(inputs.size());
  // Optimize parameters with CMAES
  metaParams = Leph::LWPROptimizeParameters(inputs.size(), metaParams, trainInputs, trainOutputs, testInputs,
                                            testOutputs, 10, false);
  // Init LWPR model
  LWPR_Object model = Leph::LWPRInit(inputs.size(), metaParams);
  // Train LWMR
  for (size_t i = 0; i < trainInputs.size(); i++)
  {
    model.update(trainInputs[i], trainOutputs[i]);
  }
  // Test LWPR
  double mse = 0.0;
  for (size_t i = 0; i < testInputs.size(); i++)
  {
    Eigen::VectorXd yp = model.predict(testInputs[i]);
    mse += pow(yp(0) - testOutputs[i], 2);
  }
  mse /= (double)testInputs.size();
  // Display informations
  std::cout << "MeanError: " << sqrt(mse) << std::endl;
  Leph::LWPRPrint(model);
  // Display prediction
  displayTest(model, output, inputs, container);

  return 0;
}
