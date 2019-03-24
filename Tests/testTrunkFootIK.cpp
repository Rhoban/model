#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Spline/CubicSpline.hpp"
#include "Plot/Plot.hpp"
#include "Utils/AxisAngle.h"

/**
 * Check given Eigen::Vector equality
 */
static void test(const std::string& msg, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  if ((v1 - v2).norm() > 0.005)
  {
    std::cout << "Test Vector Error: " << std::endl;
    std::cout << msg << std::endl;
    std::cout << "error: " << (v1 - v2).norm() << std::endl;
    std::cout << "v1: " << v1.transpose() << std::endl;
    std::cout << "v2: " << v2.transpose() << std::endl;
  }
}
static void test(const std::string& msg, const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, double epsilon)
{
  if ((v1 - v2).lpNorm<Eigen::Infinity>() > epsilon)
  {
    std::cout << "Test Vector Error: " << std::endl;
    std::cout << msg << std::endl;
    std::cout << "error: " << (v1 - v2).lpNorm<Eigen::Infinity>() << std::endl;
    std::cout << "v1: " << v1.transpose() << std::endl;
    std::cout << "v2: " << v2.transpose() << std::endl;
  }
}

int main()
{
  // Declare Humanoid model
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  model.get().setDOF("base_x", 0.1);
  model.get().setDOF("base_y", 0.1);
  model.get().setDOF("base_yaw", 0.3);
  Leph::ModelViewer viewer(1200, 900);

  // Declare target splines
  Leph::CubicSpline targetTrunkPosX;
  Leph::CubicSpline targetTrunkPosY;
  Leph::CubicSpline targetTrunkPosZ;
  Leph::CubicSpline targetTrunkAxisX;
  Leph::CubicSpline targetTrunkAxisY;
  Leph::CubicSpline targetTrunkAxisZ;
  Leph::CubicSpline targetFootPosX;
  Leph::CubicSpline targetFootPosY;
  Leph::CubicSpline targetFootPosZ;
  Leph::CubicSpline targetFootAxisX;
  Leph::CubicSpline targetFootAxisY;
  Leph::CubicSpline targetFootAxisZ;
  targetTrunkPosX.addPoint(0.0, -0.05, 0.0);
  targetTrunkPosX.addPoint(3.0, 0.0, 0.0);
  targetTrunkPosX.addPoint(6.0, 0.05, 0.0);
  targetTrunkPosY.addPoint(0.0, -0.02, 0.0);
  targetTrunkPosY.addPoint(3.0, 0.0, 0.01);
  targetTrunkPosY.addPoint(6.0, 0.03, 0.0);
  targetTrunkPosZ.addPoint(0.0, 0.15, 0.0);
  targetTrunkPosZ.addPoint(3.0, 0.2, -0.01);
  targetTrunkPosZ.addPoint(6.0, 0.17, 0.0);
  targetTrunkAxisX.addPoint(0.0, 0.0, 0.0);
  targetTrunkAxisX.addPoint(3.0, 0.2, 0.2);
  targetTrunkAxisX.addPoint(6.0, -0.4, 0.0);
  targetTrunkAxisY.addPoint(0.0, -0.3, 0.0);
  targetTrunkAxisY.addPoint(3.0, 0.1, 0.0);
  targetTrunkAxisY.addPoint(6.0, 0.2, 0.0);
  targetTrunkAxisZ.addPoint(0.0, 0.0, 0.0);
  targetTrunkAxisZ.addPoint(3.0, 0.1, 0.0);
  targetTrunkAxisZ.addPoint(6.0, -0.7, 0.0);
  targetFootPosX.addPoint(0.0, 0.02, 0.0);
  targetFootPosX.addPoint(3.0, 0.01, 0.0);
  targetFootPosX.addPoint(6.0, -0.02, 0.0);
  targetFootPosY.addPoint(0.0, -0.08, 0.0);
  targetFootPosY.addPoint(3.0, -0.04, 0.1);
  targetFootPosY.addPoint(6.0, -0.1, 0.0);
  targetFootPosZ.addPoint(0.0, 0.0, 0.0);
  targetFootPosZ.addPoint(3.0, 0.05, 0.0);
  targetFootPosZ.addPoint(6.0, -0.02, 0.0);
  targetFootAxisX.addPoint(0.0, 0.0, 0.0);
  targetFootAxisX.addPoint(6.0, 0.0, 0.0);
  targetFootAxisY.addPoint(0.0, 0.5, 0.0);
  targetFootAxisY.addPoint(3.0, -0.6, 0.0);
  targetFootAxisY.addPoint(6.0, 0.4, 0.0);
  targetFootAxisZ.addPoint(0.0, 0.0, 0.0);
  targetFootAxisZ.addPoint(6.0, 0.0, 0.0);

  Leph::Plot plot;
  double t = 0.0;
  Eigen::VectorXd oldQ(model.get().sizeDOF());
  Eigen::VectorXd oldQDiff(model.get().sizeDOF());
  while (viewer.update() && t < 6.0)
  {
    // Compute target positions
    Eigen::Vector3d trunkPos;
    Eigen::Vector3d trunkAxis;
    Eigen::Vector3d footPos;
    Eigen::Vector3d footAxis;
    trunkPos(0) = targetTrunkPosX.pos(t);
    trunkPos(1) = targetTrunkPosY.pos(t);
    trunkPos(2) = targetTrunkPosZ.pos(t);
    trunkAxis(0) = targetTrunkAxisX.pos(t);
    trunkAxis(1) = targetTrunkAxisY.pos(t);
    trunkAxis(2) = targetTrunkAxisZ.pos(t);
    footPos(0) = targetFootPosX.pos(t);
    footPos(1) = targetFootPosY.pos(t);
    footPos(2) = targetFootPosZ.pos(t);
    footAxis(0) = targetFootAxisX.pos(t);
    footAxis(1) = targetFootAxisY.pos(t);
    footAxis(2) = targetFootAxisZ.pos(t);
    // Compute target velocities
    Eigen::Vector3d trunkVel;
    Eigen::Vector3d trunkAxisVel;
    Eigen::Vector3d footVel;
    Eigen::Vector3d footAxisVel;
    trunkVel(0) = targetTrunkPosX.vel(t);
    trunkVel(1) = targetTrunkPosY.vel(t);
    trunkVel(2) = targetTrunkPosZ.vel(t);
    trunkAxisVel(0) = targetTrunkAxisX.vel(t);
    trunkAxisVel(1) = targetTrunkAxisY.vel(t);
    trunkAxisVel(2) = targetTrunkAxisZ.vel(t);
    footVel(0) = targetFootPosX.vel(t);
    footVel(1) = targetFootPosY.vel(t);
    footVel(2) = targetFootPosZ.vel(t);
    footAxisVel(0) = targetFootAxisX.vel(t);
    footAxisVel(1) = targetFootAxisY.vel(t);
    footAxisVel(2) = targetFootAxisZ.vel(t);
    // Compute target accelerations
    Eigen::Vector3d trunkAcc;
    Eigen::Vector3d trunkAxisAcc;
    Eigen::Vector3d footAcc;
    Eigen::Vector3d footAxisAcc;
    trunkAcc(0) = targetTrunkPosX.acc(t);
    trunkAcc(1) = targetTrunkPosY.acc(t);
    trunkAcc(2) = targetTrunkPosZ.acc(t);
    trunkAxisAcc(0) = targetTrunkAxisX.acc(t);
    trunkAxisAcc(1) = targetTrunkAxisY.acc(t);
    trunkAxisAcc(2) = targetTrunkAxisZ.acc(t);
    footAcc(0) = targetFootPosX.acc(t);
    footAcc(1) = targetFootPosY.acc(t);
    footAcc(2) = targetFootPosZ.acc(t);
    footAxisAcc(0) = targetFootAxisX.acc(t);
    footAxisAcc(1) = targetFootAxisY.acc(t);
    footAxisAcc(2) = targetFootAxisZ.acc(t);
    // Set IK positions
    bool isSuccess = model.trunkFootIK(Leph::HumanoidFixedModel::LeftSupportFoot, trunkPos,
                                       Leph::AxisToMatrix(trunkAxis), footPos, Leph::AxisToMatrix(footAxis));
    // Check IK success
    if (!isSuccess)
    {
      std::cout << "IK ERROR" << std::endl;
      return 1;
    }
    // Position test
    test("trunkPos", trunkPos, model.get().position("trunk", "left_foot_tip"));
    test("trunkAxis", trunkAxis, Leph::MatrixToAxis(model.get().orientation("trunk", "left_foot_tip").transpose()));
    test("footPos", footPos, model.get().position("right_foot_tip", "left_foot_tip"));
    test("footAxis", footAxis,
         Leph::MatrixToAxis(model.get().orientation("right_foot_tip", "left_foot_tip").transpose()));
    // Compute joints velocities
    Eigen::VectorXd dq = model.trunkFootIKVel(trunkVel, Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel), footVel,
                                              Leph::AxisDiffToAngularDiff(footAxis, footAxisVel));
    // Test joints velocities
    Eigen::VectorXd q = model.get().getDOFVect();
    if (t == 0.0)
      oldQ = q;
    Eigen::VectorXd qDiff = (q - oldQ) / 0.01;
    test("jointVel t=" + std::to_string(t), dq, qDiff, 0.02);
    // Compute joints accelerations
    Eigen::VectorXd ddq = model.trunkFootIKAcc(dq, trunkVel, Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel),
                                               footVel, Leph::AxisDiffToAngularDiff(footAxis, footAxisVel), trunkAcc,
                                               Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisAcc), footAcc,
                                               Leph::AxisDiffToAngularDiff(footAxis, footAxisAcc));
    // Test joints accelerations (not near acceleration discontinuities)
    if (t == 0.0)
      oldQDiff = qDiff;
    Eigen::VectorXd qDiffDiff = (qDiff - oldQDiff) / 0.01;
    if (t > 0.02 && t < 5.98 && !(t > 2.9 && t < 3.1))
    {
      test("jointAcc t=" + std::to_string(t), ddq, qDiffDiff, 0.05);
    }
    // Plot
    plot.add(Leph::VectorLabel("t", t, "vel_left:target_hip_yaw", qDiff(model.get().getDOFIndex("left_hip_yaw")),
                               "vel_left:target_hip_pitch", qDiff(model.get().getDOFIndex("left_hip_pitch")),
                               "vel_left:target_hip_roll", qDiff(model.get().getDOFIndex("left_hip_roll")),
                               "vel_left:get_hip_yaw", dq(model.get().getDOFIndex("left_hip_yaw")),
                               "vel_left:get_hip_pitch", dq(model.get().getDOFIndex("left_hip_pitch")),
                               "vel_left:get_hip_roll", dq(model.get().getDOFIndex("left_hip_roll")),
                               "vel_right:target_hip_yaw", qDiff(model.get().getDOFIndex("right_hip_yaw")),
                               "vel_right:target_hip_pitch", qDiff(model.get().getDOFIndex("right_hip_pitch")),
                               "vel_right:target_hip_roll", qDiff(model.get().getDOFIndex("right_hip_roll")),
                               "vel_right:get_hip_yaw", dq(model.get().getDOFIndex("right_hip_yaw")),
                               "vel_right:get_hip_pitch", dq(model.get().getDOFIndex("right_hip_pitch")),
                               "vel_right:get_hip_roll", dq(model.get().getDOFIndex("right_hip_roll")),
                               "acc_left:target_hip_yaw", qDiffDiff(model.get().getDOFIndex("left_hip_yaw")),
                               "acc_left:target_hip_pitch", qDiffDiff(model.get().getDOFIndex("left_hip_pitch")),
                               "acc_left:target_hip_roll", qDiffDiff(model.get().getDOFIndex("left_hip_roll")),
                               "acc_left:target_knee", qDiffDiff(model.get().getDOFIndex("left_knee")),
                               "acc_left:target_ankle_pitch", qDiffDiff(model.get().getDOFIndex("left_ankle_pitch")),
                               "acc_left:target_ankle_roll", qDiffDiff(model.get().getDOFIndex("left_ankle_roll")),
                               "acc_left:get_hip_yaw", ddq(model.get().getDOFIndex("left_hip_yaw")),
                               "acc_left:get_hip_pitch", ddq(model.get().getDOFIndex("left_hip_pitch")),
                               "acc_left:get_hip_roll", ddq(model.get().getDOFIndex("left_hip_roll")),
                               "acc_left:get_knee", ddq(model.get().getDOFIndex("left_knee")),
                               "acc_left:get_ankle_pitch", ddq(model.get().getDOFIndex("left_ankle_pitch")),
                               "acc_left:get_ankle_roll", ddq(model.get().getDOFIndex("left_ankle_roll")),
                               "acc_right:target_hip_yaw", qDiffDiff(model.get().getDOFIndex("right_hip_yaw")),
                               "acc_right:target_hip_pitch", qDiffDiff(model.get().getDOFIndex("right_hip_pitch")),
                               "acc_right:target_hip_roll", qDiffDiff(model.get().getDOFIndex("right_hip_roll")),
                               "acc_right:target_knee", qDiffDiff(model.get().getDOFIndex("right_knee")),
                               "acc_right:target_ankle_pitch", qDiffDiff(model.get().getDOFIndex("right_ankle_pitch")),
                               "acc_right:target_ankle_roll", qDiffDiff(model.get().getDOFIndex("right_ankle_roll")),
                               "acc_right:get_hip_yaw", ddq(model.get().getDOFIndex("right_hip_yaw")),
                               "acc_right:get_hip_pitch", ddq(model.get().getDOFIndex("right_hip_pitch")),
                               "acc_right:get_hip_roll", ddq(model.get().getDOFIndex("right_hip_roll")),
                               "acc_right:get_knee", ddq(model.get().getDOFIndex("right_knee")),
                               "acc_right:get_ankle_pitch", ddq(model.get().getDOFIndex("right_ankle_pitch")),
                               "acc_right:get_ankle_roll", ddq(model.get().getDOFIndex("right_ankle_roll"))));
    // Display model
    Leph::ModelDraw(model.get(), viewer);
    t += 0.01;
    oldQ = q;
    oldQDiff = qDiff;
  }
  plot.plot("t", "vel_left:*").render();
  plot.plot("t", "vel_right:*").render();
  plot.plot("t", "acc_left:*").render();
  plot.plot("t", "acc_right:*").render();

  return 0;
}
