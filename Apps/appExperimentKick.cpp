#include <iostream>
#include <vector>
#include <string>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Plot/Plot.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Utils/LWPRInputsOptimization.hpp"

static void assignModelState(Leph::HumanoidFixedPressureModel& model, const Leph::VectorLabel& log)
{
  // Asign DOF
  model.get().setDOF(log.extract("pos").rename("pos", ""), false);
  // Assiging pressure
  model.setPressure(log("pressure:weight"), log("pressure:left_ratio"), log("pressure:right_ratio"),
                    log("pressure:left_x"), log("pressure:left_y"), log("pressure:right_x"), log("pressure:right_y"));
  // Contraint the model on the ground, integrate movement
  model.updateBase();
  model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
  model.get().setDOF("base_x", 0.0);
  model.get().setDOF("base_y", 0.0);
  // Disable odometry integration
  /* TODO
  if (log("state") == 0 || model.getSupportFoot() == Leph::HumanoidFixedModel::LeftSupportFoot) {
      model.get().setDOF("base_x", 0.0);
      model.get().setDOF("base_y", 0.0);
      model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
  }
  */
  // Set IMU data for motors real model state
  model.setOrientation(Eigen::AngleAxisd(log("sensor:roll"), Eigen::Vector3d::UnitX()).toRotationMatrix() *
                           Eigen::AngleAxisd(log("sensor:pitch"), Eigen::Vector3d::UnitY()).toRotationMatrix(),
                       false);
}

/**
 * Render given logs with given viewer
 */
/*
static void displayState(Leph::ModelViewer& viewer,
    const Leph::VectorLabel& log, bool loop = false)
{
    Leph::HumanoidFixedPressureModel model(Leph::SigmabanModel);
    assignModelState(model, log);
    //Viewer loop
    Leph::Scheduling scheduling;
    scheduling.setFrequency(50.0);
    while (viewer.update()) {
        //Waiting
        scheduling.wait();
        //Model viewer
        Eigen::Vector3d copLeft = model.centerOfPressureLeft("origin");
        Eigen::Vector3d copRight = model.centerOfPressureRight("origin");
        Eigen::Vector3d copMiddle = model.centerOfPressure("origin");
        //Display foot pressure force
        double weightLeft = model.pressureWeight()
            *model.pressureLeftRatio();
        double weightRight = model.pressureWeight()
            *model.pressureRightRatio();
        double boxLength = 0.02;
        viewer.drawBox(0.005, 0.005, boxLength*weightLeft,
            copLeft + Eigen::Vector3d(0, 0, boxLength*weightLeft),
            Eigen::Matrix3d::Identity(),
            1.0, 0.0, 0.0);
        viewer.drawBox(0.005, 0.005, boxLength*weightRight,
            copRight + Eigen::Vector3d(0, 0, boxLength*weightRight),
            Eigen::Matrix3d::Identity(),
            0.0, 1.0, 0.0);
        //Display centers of pressures trajectory
        viewer.addTrackedPoint(
            copLeft, Leph::ModelViewer::Red);
        viewer.addTrackedPoint(
            copRight, Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            copMiddle, Leph::ModelViewer::Yellow);
        Leph::ModelDraw(model.get(), viewer);
        if (loop) {
            break;
        }
    }
}
*/

/**
 * Compute and add model data to given MatrixLabel
 */
static void computeModelData(Leph::MatrixLabel& logs)
{
  Leph::HumanoidFixedPressureModel model(Leph::SigmabanModel);
  for (size_t i = 10; i < logs.size(); i++)
  {
    assignModelState(model, logs[i]);
    // Data computation
    logs[i].append("model:trunk_x", model.get().position("trunk", "origin").x());
    logs[i].append("model:trunk_y", model.get().position("trunk", "origin").y());
    logs[i].append("model:trunk_z", model.get().position("trunk", "origin").z());
    logs[i].append("model:com_x", model.get().centerOfMass("origin").x());
    logs[i].append("model:com_y", model.get().centerOfMass("origin").y());
    logs[i].append("model:com_z", model.get().centerOfMass("origin").z());
    logs[i].append("model:left_foot_x", model.get().position("left_foot_tip", "origin").x());
    logs[i].append("model:left_foot_y", model.get().position("left_foot_tip", "origin").y());
    logs[i].append("model:left_foot_z", model.get().position("left_foot_tip", "origin").z());
    logs[i].append("model:right_foot_x", model.get().position("right_foot_tip", "origin").x());
    logs[i].append("model:right_foot_y", model.get().position("right_foot_tip", "origin").y());
    logs[i].append("model:right_foot_z", model.get().position("right_foot_tip", "origin").z());
    logs[i].append("model:left_cop_x", model.centerOfPressureLeft("origin").x());
    logs[i].append("model:left_cop_y", model.centerOfPressureLeft("origin").y());
    logs[i].append("model:left_cop_z", model.centerOfPressureLeft("origin").z());
    logs[i].append("model:right_cop_x", model.centerOfPressureRight("origin").x());
    logs[i].append("model:right_cop_y", model.centerOfPressureRight("origin").y());
    logs[i].append("model:right_cop_z", model.centerOfPressureRight("origin").z());
    logs[i].append("model:left_weight", model.pressureLeftWeight());
    logs[i].append("model:right_weight", model.pressureRightWeight());
    // Compute motor values
    std::vector<std::string> motorNames = {
      "left_hip_yaw",  "left_hip_pitch",  "left_hip_roll",  "left_knee",  "left_ankle_roll",  "left_ankle_pitch",
      "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_roll", "right_ankle_pitch",
    };
    for (size_t j = 0; j < motorNames.size(); j++)
    {
      logs[i].append("error:" + motorNames[j], logs[i]("goal:" + motorNames[j]) - logs[i]("pos:" + motorNames[j]));
      logs[i].append("deltapos:" + motorNames[j],
                     logs[i]("pos:" + motorNames[j]) - logs[i - 9]("pos:" + motorNames[j]));
      logs[i].append("deltagoal:" + motorNames[j],
                     logs[i]("goal:" + motorNames[j]) - logs[i - 9]("goal:" + motorNames[j]));
    }
  }
}

int main()
{
  Leph::ModelViewer viewer(1200, 900);
  viewer.maxTrajectory = 50;

  Leph::MatrixLabel logsTarget;
  Leph::MatrixLabel logsRandom;
  Leph::MatrixLabel logsRandomSeqs;
  // logsTarget.load("../../These/Data/model_2015-11-06-13-14-10.log");
  logsRandom.load("../../These/Data/model_2015-11-10-22-10-18.log");
  // logsRandom.load("/tmp/model_2015-11-10-18-38-46.log");
  // logsRandom.load("../../These/Data/model_2015-11-06-21-21-55.log");
  std::cout << "Computing Model data" << std::endl;
  // computeModelData(logsTarget);
  computeModelData(logsRandom);

  /*
  logsRandom.plot()
      .plot("time:timestamp", "pos:left_ankle_roll")
      .plot("time:timestamp", "pos:left_ankle_pitch")
      .plot("time:timestamp", "pos:left_knee")
      .plot("time:timestamp", "pos:left_hip_pitch")
      .plot("time:timestamp", "pos:left_hip_roll")
      .plot("time:timestamp", "pos:left_hip_yaw")
      .plot("time:timestamp", "goal:left_ankle_roll")
      .plot("time:timestamp", "goal:left_ankle_pitch")
      .plot("time:timestamp", "goal:left_knee")
      .plot("time:timestamp", "goal:left_hip_pitch")
      .plot("time:timestamp", "goal:left_hip_roll")
      .plot("time:timestamp", "goal:left_hip_yaw")
      .render();
  logsRandom.plot()
      //.plot("time:timestamp", "model:left_weight")
      //.plot("time:timestamp", "model:right_weight")
      .plot("time:timestamp", "model:trunk_x")
      .plot("time:timestamp", "model:trunk_y")
      .plot("time:timestamp", "model:trunk_z")
      //.plot("time:timestamp", "model:left_cop_x")
      //.plot("time:timestamp", "model:left_cop_y")
      .plot("time:timestamp", "pos:left_ankle_roll")
      .plot("time:timestamp", "pos:left_ankle_pitch")
      .plot("time:timestamp", "pos:left_knee")
      .plot("time:timestamp", "pos:left_hip_pitch")
      .plot("time:timestamp", "pos:left_hip_roll")
      .plot("time:timestamp", "pos:left_hip_yaw")
      .render();
  for (size_t i=0;i<logsRandom.size();i++) {
      std::cout << i << std::endl;
      displayState(viewer, logsRandom[i], true);
  }
  return 0;
  */

  /*
  for (size_t i=0;i<logsRandom.size();i++) {
      std::cout << i << std::endl;
      displayState(viewer, logsRandom[i], true);
  }
  */

  std::vector<std::pair<size_t, size_t>> seqs;
  std::pair<size_t, size_t> seq;
  for (size_t i = 1; i < logsRandom.size(); i++)
  {
    if (logsRandom[i]("state") >= 1 && logsRandom[i]("state") <= 3 &&
        (logsRandom[i - 1]("state") == 0 || logsRandom[i - 1]("state") == 4))
    {
      seq.first = i;
    }
    if ((logsRandom[i]("state") == 0 || logsRandom[i]("state") == 4) && logsRandom[i - 1]("state") >= 1 &&
        logsRandom[i - 1]("state") <= 3)
    {
      seq.second = i - 1;
      seqs.push_back(seq);
    }
  }
  for (size_t i = 0; i < seqs.size(); i++)
  {
    for (size_t j = seqs[i].first; j <= seqs[i].second; j++)
    {
      logsRandomSeqs.append(logsRandom[j]);
    }
  }
  std::cout << "Sequences " << seqs.size() << std::endl;

  // Test optimization TODO TODO TODO TODO
  Leph::LWPRInputsOptimization optim(0.5, 5);
  optim.setOuput("goal:left_hip_pitch");
  std::vector<std::string> inputs;
  for (auto& str : logsRandom[20].labels())
  {
    if (str.first.find("goal:") == std::string::npos && str.first.find("head") == std::string::npos &&
        str.first.find("shoulder") == std::string::npos && str.first.find("elbow") == std::string::npos &&
        str.first.find("sensor:acc") == std::string::npos && str.first.find("target:") == std::string::npos &&
        str.first.find("rate:") == std::string::npos && str.first.find("error:") == std::string::npos &&
        str.first.find("pressure:") == std::string::npos && str.first.find("time:") == std::string::npos &&
        str.first.find("state") == std::string::npos && str.first.find("deltagoal:") == std::string::npos &&
        str.first.find("deltapos:") == std::string::npos && str.first.find("sensor:gyro") == std::string::npos)
    {
      inputs.push_back(str.first);
    }
  }
  optim.setInputs(inputs);
  for (size_t i = 0; i < seqs.size(); i++)
  {
    optim.addSequence(logsRandom.range(seqs[i].first, seqs[i].second));
  }
  optim.optimize();

  // logsRandomSeqs.plot().plot("model:com_x", "model:com_y", Leph::Plot::Points, "model:com_z").render();
  // logsRandomSeqs.plot().plot("model:left_weight", "model:right_weight").render();
  // logsRandomSeqs.plot().plot("model:left_foot_z", "model:right_foot_z").render();
  /*
  logsRandomSeqs.plot()
      .plot("time:timestamp", "pos:left_ankle_roll")
      .plot("time:timestamp", "pos:left_ankle_pitch")
      .plot("time:timestamp", "pos:left_knee")
      .plot("time:timestamp", "pos:left_hip_pitch")
      .plot("time:timestamp", "pos:left_hip_roll")
      .plot("time:timestamp", "pos:left_hip_yaw")
      .plot("time:timestamp", "goal:left_ankle_roll")
      .plot("time:timestamp", "goal:left_ankle_pitch")
      .plot("time:timestamp", "goal:left_knee")
      .plot("time:timestamp", "goal:left_hip_pitch")
      .plot("time:timestamp", "goal:left_hip_roll")
      .plot("time:timestamp", "goal:left_hip_yaw")
      .render();
  logsRandomSeqs.plot()
      .plot("time:timestamp", "model:left_weight")
      .render();
  logsRandomSeqs.plot()
      .plot("time:timestamp", "model:left_cop_x")
      .plot("time:timestamp", "model:left_cop_y")
      .plot("time:timestamp", "model:right_cop_x")
      .plot("time:timestamp", "model:right_cop_y")
      .render();
  logsRandomSeqs.plot()
      .plot("time:timestamp", "model:trunk_x")
      .plot("time:timestamp", "model:trunk_y")
      .plot("time:timestamp", "pos:left_ankle_roll")
      .plot("time:timestamp", "pos:left_ankle_pitch")
      .plot("time:timestamp", "pos:left_knee")
      .plot("time:timestamp", "pos:left_hip_pitch")
      .plot("time:timestamp", "pos:left_hip_roll")
      .plot("time:timestamp", "pos:left_hip_yaw")
      .render();
  */
  /*
  logsRandomSeqs.plot()
      .plot("deltagoal:right_ankle_roll", "deltapos:right_ankle_roll", Leph::Plot::Points)
      .render();
  */

  /*
  displayState(viewer, logsRandom[1740]);
  displayState(viewer, logsRandom[15300]);
  displayState(viewer, logsRandom[21245]);
  */
  /*
  for (size_t i=11000;i<12200;i++) {
      std::cout << i << std::endl;
      displayState(viewer, logsRandomSeqs[i], true);
  }
  */

  return 0;
}
