#include <iostream>
#include "Types/MapSeries.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Spline/PolyFit.hpp"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: ./app [log.series]" << std::endl;
    return 1;
  }

  // Load data
  std::string logFileName = argv[1];
  std::cout << "Loading " << logFileName << std::endl;
  Leph::MapSeries series;
  series.importData(logFileName);

  series.plot()
      .plot("time", "state")
      .plot("time", "phase")
      .plot("time", "pos:left_ankle_pitch")
      .plot("time", "pos:left_ankle_roll")
      .plot("time", "pos:left_knee")
      .plot("time", "goal:left_ankle_pitch")
      .plot("time", "goal:left_ankle_roll")
      .plot("time", "goal:left_knee")
      .render();

  series.plot()
      .plot("time", "pos:left_hip_pitch")
      .plot("time", "pos:left_hip_roll")
      .plot("time", "pos:left_hip_yaw")
      .plot("time", "goal:left_hip_pitch")
      .plot("time", "goal:left_hip_roll")
      .plot("time", "goal:left_hip_yaw")
      .render();

  // Initialize Model
  Leph::HumanoidFixedModel modelPos(Leph::SigmabanModel);
  Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);

  Leph::ModelViewer viewer(1200, 900);

  Leph::Plot plot;
  for (double t = series.timeMin(); t < series.timeMax(); t += 0.01)
  {
    // Compute phase and state
    Leph::MapSeries::Point ptLow;
    Leph::MapSeries::Point ptUp;
    double phase = series.get("phase", t, nullptr, nullptr, &ptLow, &ptUp);
    std::cout << t << " " << phase << " --- " << std::setprecision(10) << ptLow.time << " " << ptLow.value << " --- "
              << std::setprecision(10) << ptUp.time << " " << ptUp.value << std::endl;
    double state = series.get("state", t);
    if (state != 1.0)
    {
      continue;
    }
    // Discard point if the phase is cycling
    if (ptLow.value > ptUp.value)
    {
      continue;
    }
    // Compute DOFs position, velocity and acceleration
    Eigen::VectorXd posPos = modelPos.get().getDOFVect();
    Eigen::VectorXd velPos = modelPos.get().getDOFVect();
    Eigen::VectorXd accPos = modelPos.get().getDOFVect();
    Eigen::VectorXd posGoal = modelGoal.get().getDOFVect();
    Eigen::VectorXd velGoal = modelGoal.get().getDOFVect();
    Eigen::VectorXd accGoal = modelGoal.get().getDOFVect();
    velPos.setZero();
    accPos.setZero();
    velGoal.setZero();
    accGoal.setZero();
    for (const std::string& name : Leph::NamesDOF)
    {
      break;
      Leph::PolyFit fitPos(5);
      Leph::PolyFit fitGoal(5);
      size_t indexPos = series.getIndex("pos:" + name, t);
      size_t indexGoal = series.getIndex("pos:" + name, t);
      double meanPos = 0.0;
      double varPos = 0.0;
      double meanGoal = 0.0;
      double varGoal = 0.0;
      size_t countPos = 0;
      size_t countGoal = 0;
      size_t delta = 5;
      for (size_t i = indexPos - delta; i <= indexPos + delta; i++)
      {
        meanPos += series.at("pos:" + name, i).time;
        varPos += pow(series.at("pos:" + name, i).time, 2);
        countPos++;
      }
      for (size_t i = indexGoal - delta; i <= indexGoal + delta; i++)
      {
        meanGoal += series.at("goal:" + name, i).time;
        varGoal += pow(series.at("goal:" + name, i).time, 2);
        countGoal++;
      }
      meanPos = meanPos / (double)countPos;
      meanGoal = meanGoal / (double)countGoal;
      varPos = sqrt(varPos / (double)countPos - meanPos * meanPos);
      varGoal = sqrt(varGoal / (double)countGoal - meanGoal * meanGoal);
      for (size_t i = indexPos - delta; i <= indexPos + delta; i++)
      {
        fitPos.add((series.at("pos:" + name, i).time - meanPos) / varPos, series.at("pos:" + name, i).value);
      }
      for (size_t i = indexGoal - delta; i <= indexGoal + delta; i++)
      {
        fitGoal.add((series.at("goal:" + name, i).time - meanPos) / varPos, series.at("goal:" + name, i).value);
      }
      Leph::Polynom polyPos = fitPos.fitting();
      Leph::Polynom polyGoal = fitGoal.fitting();
      posPos(modelPos.get().getDOFIndex(name)) = polyPos.pos((t - meanPos) / varPos);
      velPos(modelPos.get().getDOFIndex(name)) = polyPos.vel((t - meanPos) / varPos) / varPos;
      accPos(modelPos.get().getDOFIndex(name)) = polyPos.acc((t - meanPos) / varPos) / (varPos * varPos);
      posGoal(modelGoal.get().getDOFIndex(name)) = polyGoal.pos((t - meanGoal) / varGoal);
      velGoal(modelGoal.get().getDOFIndex(name)) = polyGoal.vel((t - meanGoal) / varGoal) / varGoal;
      accGoal(modelGoal.get().getDOFIndex(name)) = polyGoal.acc((t - meanGoal) / varGoal) / (varGoal * varGoal);
      plot.add(Leph::VectorLabel(
          "time", t, "phase", phase, "pos:" + name, series.get("pos:" + name, t), "fitted_pos_pos:" + name,
          posPos(modelPos.get().getDOFIndex(name)), "fitted_pos_vel:" + name, velPos(modelPos.get().getDOFIndex(name)),
          "fitted_pos_acc:" + name, accPos(modelPos.get().getDOFIndex(name)), "goal:" + name,
          series.get("goal:" + name, t), "fitted_goal_pos:" + name, posGoal(modelGoal.get().getDOFIndex(name)),
          "fitted_goal_vel:" + name, velGoal(modelGoal.get().getDOFIndex(name)), "fitted_goal_acc:" + name,
          accGoal(modelGoal.get().getDOFIndex(name))));
    }
    // Update models
    modelPos.get().setDOFVect(posPos);
    modelGoal.get().setDOFVect(posGoal);
    modelPos.updateBase();
    modelGoal.updateBase();
    // Compute dynamics
    Eigen::Vector3d zmpPos = modelPos.zeroMomentPoint("origin", velPos, accPos);
    Eigen::Vector3d zmpGoal = modelGoal.zeroMomentPoint("origin", velGoal, accGoal);
    zmpPos.z() = 0.0;
    zmpGoal.z() = 0.0;
    // Display
    viewer.addTrackedPoint(zmpPos, Leph::ModelViewer::Yellow);
    viewer.addTrackedPoint(zmpGoal, Leph::ModelViewer::Green);
    // Leph::ModelDraw(modelPos.get(), viewer);
    Leph::ModelDraw(modelGoal.get(), viewer);
    viewer.update();
  }
  plot.plot("time", "phase").render();
  plot
      /*
      .plot("time", "goal:left_knee")
      .plot("time", "fitted_goal_pos:left_knee")
      .plot("time", "fitted_goal_vel:left_knee")
      .plot("time", "fitted_goal_acc:left_knee")
      */
      /*
      .plot("time", "pos:left_knee")
      .plot("time", "fitted_pos_pos:left_knee")
      .plot("time", "fitted_pos_vel:left_knee")
      .plot("time", "fitted_pos_acc:left_knee")
      */
      /*
      .plot("phase", "goal:left_knee", Leph::Plot::Points)
      .plot("phase", "fitted_goal_pos:left_knee", Leph::Plot::Points)
      .plot("phase", "fitted_goal_vel:left_knee", Leph::Plot::Points)
      .plot("phase", "fitted_goal_acc:left_knee", Leph::Plot::Points)
      */
      .plot("phase", "goal:left_knee", Leph::Plot::Points)
      .plot("phase", "fitted_goal_pos:left_knee", Leph::Plot::Points)
      .render();

  return 0;
}
