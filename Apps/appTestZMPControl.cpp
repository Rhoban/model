#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "Types/VectorLabel.hpp"
#include "Types/MatrixLabel.hpp"
#include "Spline/CubicSpline.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Spline/FittedSpline.hpp"
#include "Spline/SplineContainer.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/AxisAngle.h"
#include "Plot/Plot.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/NamesModel.h"

/**
 * Global spline trajectory
 * for external perturbation
 */
static std::string SplinesFile = "../../These/Data/logs-2016-02-02/newTrajs/"
                                 "trajBest_3.000000_1.000000_0.000000_3.353406.splines";
static Leph::SplineContainer<Leph::CubicSpline> Splines;

/**
 * Return randomized initial position
 */
/*
Leph::VectorLabel initialGoalState()
{
    Leph::VectorLabel state;
    state.append("goal_model:trunk_pos_x", -0.0042);
    state.append("goal_model:trunk_pos_y", -0.0126);
    state.append("goal_model:trunk_pos_z", 0.275);
    state.append("goal_model:trunk_axis_x", -0.6531);
    state.append("goal_model:trunk_axis_y", 0.0566);
    state.append("goal_model:trunk_axis_z", 0.0496);
    state.append("goal_model:foot_pos_x", 0.0253);
    state.append("goal_model:foot_pos_y", -0.9);
    state.append("goal_model:foot_pos_z", 0.0570);
    state.append("goal_model:foot_axis_x", 0.0);
    state.append("goal_model:foot_axis_y", 0.0);

    return state;
}
*/

/**
 * Assign to given VectorLabel IK model goal
 * at given time t from global trajectory splines
 */
void setSplineGoals(Leph::VectorLabel& state, double t)
{
  state.setOrAppend("t", t);
  state.setOrAppend("goal_model:trunk_pos_x", Splines.get("trunk_pos_x").pos(t));
  state.setOrAppend("goal_model:trunk_pos_y", Splines.get("trunk_pos_y").pos(t));
  state.setOrAppend("goal_model:trunk_pos_z", Splines.get("trunk_pos_z").pos(t));
  state.setOrAppend("goal_model:trunk_axis_x", Splines.get("trunk_axis_x").pos(t));
  state.setOrAppend("goal_model:trunk_axis_y", Splines.get("trunk_axis_y").pos(t));
  state.setOrAppend("goal_model:trunk_axis_z", Splines.get("trunk_axis_z").pos(t));
  state.setOrAppend("goal_model:foot_pos_x", Splines.get("foot_pos_x").pos(t));
  state.setOrAppend("goal_model:foot_pos_y", Splines.get("foot_pos_y").pos(t));
  state.setOrAppend("goal_model:foot_pos_z", Splines.get("foot_pos_z").pos(t));
  state.setOrAppend("goal_model:foot_axis_x", Splines.get("foot_axis_x").pos(t));
  state.setOrAppend("goal_model:foot_axis_y", Splines.get("foot_axis_y").pos(t));
  state.setOrAppend("goal_model:foot_axis_z", Splines.get("foot_axis_z").pos(t));
}

/**
 * Compute DOF goal using inverse.
 * False is returned if IK ERROR
 */
bool setDOFGoals(Leph::VectorLabel& state)
{
  // Retrieve cartesian position
  Eigen::Vector3d trunkPos(state("goal_model:trunk_pos_x"), state("goal_model:trunk_pos_y"),
                           state("goal_model:trunk_pos_z"));
  Eigen::Vector3d trunkAxis(state("goal_model:trunk_axis_x"), state("goal_model:trunk_axis_y"),
                            state("goal_model:trunk_axis_z"));
  Eigen::Vector3d footPos(state("goal_model:foot_pos_x"), state("goal_model:foot_pos_y"),
                          state("goal_model:foot_pos_z"));
  Eigen::Vector3d footAxis(state("goal_model:foot_axis_x"), state("goal_model:foot_axis_y"),
                           state("goal_model:foot_axis_z"));
  // Inverse kinematics
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  bool isSuccess = model.trunkFootIK(Leph::HumanoidFixedModel::LeftSupportFoot, trunkPos, Leph::AxisToMatrix(trunkAxis),
                                     footPos, Leph::AxisToMatrix(footAxis));
  if (!isSuccess)
  {
    std::cout << "IK ERROR GOAL " << state("t") << std::endl;
    return false;
  }
  // Assign DOFs
  state.mergeUnion(model.get().getDOF().rename("", "goal"));
  return true;
}

/**
 * Compute for index the actual read DOF position
 * from history of goal positions
 */
void computeDOFPos(Leph::MatrixLabel mat, size_t index)
{
  if (index < 2)
  {
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      mat[index].setOrAppend("pos:" + name, mat[index]("goal:" + name));
    }
  }
  else
  {
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      mat[index].setOrAppend("pos:" + name, 0.6 * mat[index - 1]("pos:" + name) + 0.4 * mat[index - 2]("goal:" + name));
    }
  }
}

/**
 * Fit the last DOF position with a spline
 * and estimate current DOF velocity and acceleration
 */
void computeDOFVelAcc(Leph::MatrixLabel mat, size_t index)
{
  if (index < 9)
  {
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      mat[index].setOrAppend("vel:" + name, 0.0);
      mat[index].setOrAppend("acc:" + name, 0.0);
    }
  }
  else
  {
    size_t len = 1;
    size_t degree = 1;
    if (index <= 3)
    {
      len = index;
      degree = index;
    }
    else if (index <= 9)
    {
      len = index;
      degree = 3;
    }
    else
    {
      len = 9;
      degree = 3;
    }
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      Leph::FittedSpline fitted;
      for (size_t i = index - len; i <= index; i++)
      {
        fitted.addPoint(mat[i]("t"), mat[i]("pos:" + name));
      }
      fitted.fittingGlobal(degree, len + 1);
      mat[index].setOrAppend("vel:" + name, fitted.vel(mat[index]("t")));
      mat[index].setOrAppend("acc:" + name, fitted.acc(mat[index]("t")));
    }
  }
  mat[index].setOrAppend("pos:base_x", 0.0);
  mat[index].setOrAppend("pos:base_y", 0.0);
  mat[index].setOrAppend("pos:base_z", 0.0);
  mat[index].setOrAppend("pos:base_pitch", 0.0);
  mat[index].setOrAppend("pos:base_roll", 0.0);
  mat[index].setOrAppend("pos:base_yaw", 0.0);
  mat[index].setOrAppend("vel:base_x", 0.0);
  mat[index].setOrAppend("vel:base_y", 0.0);
  mat[index].setOrAppend("vel:base_z", 0.0);
  mat[index].setOrAppend("vel:base_pitch", 0.0);
  mat[index].setOrAppend("vel:base_roll", 0.0);
  mat[index].setOrAppend("vel:base_yaw", 0.0);
  mat[index].setOrAppend("acc:base_x", 0.0);
  mat[index].setOrAppend("acc:base_y", 0.0);
  mat[index].setOrAppend("acc:base_z", 0.0);
  mat[index].setOrAppend("acc:base_pitch", 0.0);
  mat[index].setOrAppend("acc:base_roll", 0.0);
  mat[index].setOrAppend("acc:base_yaw", 0.0);
}

/**
 * Compute from DOF state model estimation
 */
void computeModel(Leph::VectorLabel& state)
{
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  model.get().setDOF(state.extract("pos").rename("pos", ""), false);
  model.updateBase();
  state.setOrAppend("model:com_x", model.get().centerOfMass("left_foot_tip").x());
  state.setOrAppend("model:com_y", model.get().centerOfMass("left_foot_tip").y());
  state.setOrAppend("model:com_z", model.get().centerOfMass("left_foot_tip").z());
  state.setOrAppend("model:trunk_pos_x", model.get().position("trunk", "left_foot_tip").x());
  state.setOrAppend("model:trunk_pos_y", model.get().position("trunk", "left_foot_tip").y());
  state.setOrAppend("model:trunk_pos_z", model.get().position("trunk", "left_foot_tip").z());
  state.setOrAppend("model:foot_pos_x", model.get().position("right_foot_tip", "left_foot_tip").x());
  state.setOrAppend("model:foot_pos_y", model.get().position("right_foot_tip", "left_foot_tip").y());
  state.setOrAppend("model:foot_pos_z", model.get().position("right_foot_tip", "left_foot_tip").z());
  state.setOrAppend("model:trunk_axis_x",
                    Leph::MatrixToAxis(model.get().orientation("trunk", "left_foot_tip").transpose()).x());
  state.setOrAppend("model:trunk_axis_y",
                    Leph::MatrixToAxis(model.get().orientation("trunk", "left_foot_tip").transpose()).y());
  state.setOrAppend("model:trunk_axis_z",
                    Leph::MatrixToAxis(model.get().orientation("trunk", "left_foot_tip").transpose()).z());
  state.setOrAppend("model:foot_axis_x",
                    Leph::MatrixToAxis(model.get().orientation("right_foot_tip", "left_foot_tip").transpose()).x());
  state.setOrAppend("model:foot_axis_y",
                    Leph::MatrixToAxis(model.get().orientation("right_foot_tip", "left_foot_tip").transpose()).y());
  state.setOrAppend("model:foot_axis_z",
                    Leph::MatrixToAxis(model.get().orientation("right_foot_tip", "left_foot_tip").transpose()).z());

  Leph::VectorLabel torques =
      model.get().inverseDynamics(state.extract("vel").rename("vel", ""), state.extract("acc").rename("acc", ""));
  state.mergeUnion(torques.rename("", "torque"));
  Eigen::Vector3d zmp /* XXX TODO API change = model.zeroMomentPoint("left_foot_tip",
      state.extract("vel").rename("vel", ""),
      state.extract("acc").rename("acc", "")) TODO XXX */
      = Eigen::Vector3d(0, 0, 0);
  state.setOrAppend("model:zmp_x", zmp.x());
  state.setOrAppend("model:zmp_y", zmp.y());

  torques("base_x") = 0.0;
  torques("base_y") = 0.0;
  torques("base_z") = 0.0;
  torques("base_yaw") = 0.0;
  torques("base_pitch") = 0.0;
  torques("base_roll") = 0.0;
  state.setOrAppend("torque:norm_2", torques.vect().norm());
  state.setOrAppend("torque:norm_inf", torques.vect().lpNorm<Eigen::Infinity>());

  state.setOrAppend("model:pressure_x", zmp.x());
  state.setOrAppend("model:pressure_y", zmp.y());

  if (state("model:pressure_x") > 0.6)
    state("model:pressure_x") = 0.6;
  if (state("model:pressure_x") < -0.6)
    state("model:pressure_x") = -0.6;
  if (state("model:pressure_y") > 0.4)
    state("model:pressure_y") = 0.4;
  if (state("model:pressure_y") < -0.4)
    state("model:pressure_y") = -0.4;
}

void control(Leph::MatrixLabel& states, size_t index)
{
  if (index == 0)
  {
    states[index].setOrAppend("control:x", 0.0);
    return;
  }

  /*
  states[index].setOrAppend("control:x",
      -0.005*states[index-1]("model:pressure_x")
      + states[index-1]("control:x"));
  states[index]("goal_model:trunk_pos_x") += states[index]("control:x");
  */
  /*
  states[index].setOrAppend("control:x", -0.01*states[index-1]("model:pressure_x"));
  states[index]("goal_model:trunk_pos_x") += states[index]("control:x");
  */
}

int main()
{
  // Create global trajectory splines
  Splines.add("trunk_pos_x");
  Splines.add("trunk_pos_y");
  Splines.add("trunk_pos_z");
  Splines.add("trunk_axis_x");
  Splines.add("trunk_axis_y");
  Splines.add("trunk_axis_z");
  Splines.add("foot_pos_x");
  Splines.add("foot_pos_y");
  Splines.add("foot_pos_z");
  Splines.add("foot_axis_x");
  Splines.add("foot_axis_y");
  Splines.add("foot_axis_z");

  double tStart1 = 1.5;
  double tStop1 = tStart1 + 0.5;
  double tMiddle1 = 0.5 * tStart1 + 0.5 * tStop1;

  double tStart2 = tStop1 + 1.0;
  double tStop2 = tStart2 + 0.5;
  double tMiddle2 = 0.5 * tStart2 + 0.5 * tStop2;

  double tStart3 = tStop2 + 1.0;
  double tStop3 = tStart3 + 0.5;
  double tMiddle3 = 0.5 * tStart3 + 0.5 * tStop3;

  double tStart4 = tStop3 + 1.0;
  double tStop4 = tStart4 + 0.5;
  double tMiddle4 = 0.5 * tStart4 + 0.5 * tStop4;

  double tEnd = tStop4 + 1.0;
  double length = 0.01;
  Splines.get("trunk_pos_x").addPoint(0.0, -0.0042);

  Splines.get("trunk_pos_x").addPoint(tStart1, -0.0042);
  Splines.get("trunk_pos_x").addPoint(tMiddle1, -0.0042 + length);
  Splines.get("trunk_pos_x").addPoint(tStop1, -0.0042);

  Splines.get("trunk_pos_x").addPoint(tStart3, -0.0042);
  Splines.get("trunk_pos_x").addPoint(tMiddle3, -0.0042 + length);
  Splines.get("trunk_pos_x").addPoint(tStop3, -0.0042);
  Splines.get("trunk_pos_x").addPoint(tStart4, -0.0042);
  Splines.get("trunk_pos_x").addPoint(tMiddle4, -0.0042 + 2.0 * length);
  Splines.get("trunk_pos_x").addPoint(tStop4, -0.0042);

  Splines.get("trunk_pos_x").addPoint(tEnd, -0.0042);
  Splines.get("trunk_pos_y").addPoint(0.0, -0.0126);
  Splines.get("trunk_pos_y").addPoint(tEnd, -0.0126);
  Splines.get("trunk_pos_z").addPoint(0.0, 0.27);
  Splines.get("trunk_pos_z").addPoint(tEnd, 0.27);
  Splines.get("trunk_axis_x").addPoint(0.0, -0.65313);
  Splines.get("trunk_axis_x").addPoint(tEnd, -0.65313);
  Splines.get("trunk_axis_y").addPoint(0.0, 0.05665);

  Splines.get("trunk_axis_y").addPoint(tStart2, 0.05665);
  Splines.get("trunk_axis_y").addPoint(tMiddle2, 0.05665 + 25.0 * length);
  Splines.get("trunk_axis_y").addPoint(tStop2, 0.05665);
  Splines.get("trunk_axis_y").addPoint(tStart3, 0.05665);
  Splines.get("trunk_axis_y").addPoint(tMiddle3, 0.05665 + 25.0 * length);
  Splines.get("trunk_axis_y").addPoint(tStop3, 0.05665);
  Splines.get("trunk_axis_y").addPoint(tStart4, 0.05665);
  Splines.get("trunk_axis_y").addPoint(tMiddle4, 0.05665 - 50.0 * length);
  Splines.get("trunk_axis_y").addPoint(tStop4, 0.05665);

  Splines.get("trunk_axis_y").addPoint(tEnd, 0.05665);
  Splines.get("trunk_axis_z").addPoint(0.0, 0.04961);
  Splines.get("trunk_axis_z").addPoint(tEnd, 0.04961);
  Splines.get("foot_pos_x").addPoint(0.0, 0.02530);
  /*
  Splines.get("foot_pos_x").addPoint(tStart, 0.02530);
  Splines.get("foot_pos_x").addPoint(tMiddle, 0.02530 + length);
  Splines.get("foot_pos_x").addPoint(tStop, 0.02530);
  */
  Splines.get("foot_pos_x").addPoint(tEnd, 0.02530);
  Splines.get("foot_pos_y").addPoint(0.0, -0.09);
  Splines.get("foot_pos_y").addPoint(tEnd, -0.09);
  Splines.get("foot_pos_z").addPoint(0.0, 0.05706);
  Splines.get("foot_pos_z").addPoint(tEnd, 0.05706);
  Splines.get("foot_axis_x").addPoint(0.0, 0.0);
  Splines.get("foot_axis_x").addPoint(tEnd, 0.0);
  Splines.get("foot_axis_y").addPoint(0.0, 0.0);
  Splines.get("foot_axis_y").addPoint(tEnd, 0.0);
  Splines.get("foot_axis_z").addPoint(0.0, 0.0);
  Splines.get("foot_axis_z").addPoint(tEnd, 0.0);

  /*
  //Load global trajectory splines
  Splines.importData(SplinesFile);
  */

  Leph::MatrixLabel states;
  Leph::MatrixLabel statesControlled;
  for (double t = Splines.min(); t <= Splines.max(); t += 0.01)
  {
    Leph::VectorLabel goalVect;
    setSplineGoals(goalVect, t);
    states.append(goalVect);
    statesControlled.append(goalVect);
  }
  for (size_t i = 0; i < states.size(); i++)
  {
    control(statesControlled, i);
    bool isSuccess1 = setDOFGoals(states[i]);
    bool isSuccess2 = setDOFGoals(statesControlled[i]);
    if (!isSuccess1)
    {
      break;
    }
    if (!isSuccess2)
    {
      break;
    }
    computeDOFPos(states, i);
    computeDOFPos(statesControlled, i);
    computeDOFVelAcc(states, i);
    computeDOFVelAcc(statesControlled, i);
    computeModel(states[i]);
    computeModel(statesControlled[i]);
  }

  /*
  Leph::MatrixLabel statesMerged = states;
  for (size_t i=0;i<states.size();i++) {
      statesMerged.appendForce(statesControlled[i]);
  }

  statesMerged.plot()
      .plot("t", "control:*")
      .plot("t", "goal_model:trunk_pos_x")
      .plot("t", "model:pressure_x", Leph::Plot::LinesPoints, "index")
      .render();
  */

  // Display
  Leph::ModelViewer viewer(1200, 900);
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  while (true)
  {
    for (size_t i = 0; i < states.size(); i += 2)
    {
      if (!viewer.update())
      {
        goto end;
      }
      std::cout << states[i]("t") << std::endl;
      model.get().setDOF(states[i].extract("pos").rename("pos", ""));
      model.updateBase();
      viewer.addTrackedPoint(Eigen::Vector3d(states[i]("model:pressure_x"), states[i]("model:pressure_y"), 0.0),
                             Leph::ModelViewer::Yellow);
      Leph::ModelDraw(model.get(), viewer);
    }
  }
end:
  states.plot()
      .plot("t", "model:pressure_x")
      .plot("t", "model:com_x")
      .plot("t", "torque:norm_2")
      .plot("t", "torque:norm_inf")
      .render();

  return 0;
}
