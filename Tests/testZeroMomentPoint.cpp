#include <iostream>
#include "Model/HumanoidFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Spline/SmoothSpline.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

/**
 * Test single and double support
 * ZMP computation. Assert that ZMP
 * computed in single support and
 * in double support are the same.
 */
int main()
{
  // Model and viewer initialization
  Leph::ModelViewer viewer(1200, 900);
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);

  // Trajectory initialization
  Leph::SmoothSpline spline;
  spline.addPoint(0.0, -0.07);
  spline.addPoint(0.25, 0.05);
  spline.addPoint(0.5, -0.10);
  spline.addPoint(0.75, -0.07);
  spline.addPoint(1.0, -0.07);
  // Support foot
  Leph::HumanoidFixedModel::SupportFoot supportFoot = Leph::HumanoidFixedModel::LeftSupportFoot;
  // Target cartesian position
  Eigen::Vector3d trunkPos(0.0, -0.07, 0.20);
  Eigen::Vector3d trunkAxis(0.0, 0.0, 0.0);
  Eigen::Vector3d footPos(0.0, -0.14, 0.0);
  Eigen::Vector3d footAxis(0.0, 0.0, 0.0);
  // Target cartesian velocity
  Eigen::Vector3d trunkPosVel(0.0, 0.0, 0.0);
  Eigen::Vector3d trunkAxisVel(0.0, 0.0, 0.0);
  Eigen::Vector3d footPosVel(0.0, 0.0, 0.0);
  Eigen::Vector3d footAxisVel(0.0, 0.0, 0.0);
  // Target cartesian acceleration
  Eigen::Vector3d trunkPosAcc(0.0, 0.0, 0.0);
  Eigen::Vector3d trunkAxisAcc(0.0, 0.0, 0.0);
  Eigen::Vector3d footPosAcc(0.0, 0.1, 0.0);
  Eigen::Vector3d footAxisAcc(0.0, 0.0, 0.0);
  model.get().setDOF("base_x", 0.2);
  model.get().setDOF("base_y", 0.1);
  model.get().setDOF("base_yaw", 0.4);

  Leph::Plot plot;
  Leph::Scheduling scheduling(100.0);
  double t = 0.0;
  while (viewer.update())
  {
    // Update target
    trunkPos.x() = spline.posMod(t);
    trunkPos.y() = spline.posMod(t);
    trunkPosVel.x() = spline.velMod(t);
    trunkPosVel.y() = spline.velMod(t);
    trunkPosAcc.x() = spline.accMod(t);
    trunkPosAcc.y() = spline.accMod(t);
    trunkAxis.z() = 0.3;
    footAxis.z() = 0.6;
    // Compute DOF positions
    bool isSuccess =
        model.trunkFootIK(supportFoot, trunkPos, Leph::AxisToMatrix(trunkAxis), footPos, Leph::AxisToMatrix(footAxis));
    if (!isSuccess)
    {
      std::cout << "IK Error t=" << t << std::endl;
      return 1;
    }
    // Axis differentiation is converted in proper angular
    // velocity and acceleration
    // Compute DOF velocities
    Eigen::VectorXd dq = model.trunkFootIKVel(trunkPosVel, Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel),
                                              footPosVel, Leph::AxisDiffToAngularDiff(footAxis, footAxisVel));
    // Compute DOF accelerations
    Eigen::VectorXd ddq = model.trunkFootIKAcc(dq, trunkPosVel, Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel),
                                               footPosVel, Leph::AxisDiffToAngularDiff(footAxis, footAxisVel),
                                               trunkPosAcc, Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisAcc),
                                               footPosAcc, Leph::AxisDiffToAngularDiff(footAxis, footAxisAcc));
    // Convert velocity and acceleration
    Eigen::VectorXd dqLeft = dq;
    Eigen::VectorXd ddqLeft = ddq;
    Eigen::VectorXd dqRight = Eigen::VectorXd::Zero(dq.size());
    Eigen::VectorXd ddqRight = Eigen::VectorXd::Zero(ddq.size());
    for (const std::string& name : Leph::NamesDOFAll)
    {
      model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
      size_t indexLeft = model.get().getDOFIndex(name);
      model.setSupportFoot(Leph::HumanoidFixedModel::RightSupportFoot);
      size_t indexRight = model.get().getDOFIndex(name);
      dqRight(indexRight) = dqLeft(indexLeft);
      ddqRight(indexRight) = ddqLeft(indexLeft);
    }

    // Compute ZMP point
    model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
    Eigen::Vector3d zmpSingleLeft = model.zeroMomentPoint("origin", dqLeft, ddqLeft, false);
    Eigen::Vector3d zmpDoubleLeft = model.zeroMomentPoint("origin", dqLeft, ddqLeft, true);
    model.setSupportFoot(Leph::HumanoidFixedModel::RightSupportFoot);
    Eigen::Vector3d zmpSingleRight = model.zeroMomentPoint("origin", dqRight, ddqRight, false);
    Eigen::Vector3d zmpDoubleRight = model.zeroMomentPoint("origin", dqRight, ddqRight, true);
    std::cout << "ZMPSingleLeft t=" << t << " " << zmpSingleLeft.x() << " " << zmpSingleLeft.y() << std::endl;
    std::cout << "ZMPSingleRight t=" << t << " " << zmpSingleRight.x() << " " << zmpSingleRight.y() << std::endl;
    std::cout << "ZMPDoubleLeft t=" << t << " " << zmpDoubleLeft.x() << " " << zmpDoubleLeft.y() << std::endl;
    std::cout << "ZMPDoubleRight t=" << t << " " << zmpDoubleRight.x() << " " << zmpDoubleRight.y() << std::endl;
    std::cout << std::endl;
    plot.add({
        "t",
        t,
        "pos",
        trunkPos.y(),
        "vel",
        trunkPosVel.y(),
        "acc",
        trunkPosAcc.y(),
        "zmpSingleLeft",
        zmpSingleLeft.y(),
        "zmpSingleRight",
        zmpSingleRight.y(),
        "zmpDoubleLeft",
        zmpDoubleLeft.y(),
        "zmpDoubleRight",
        zmpDoubleRight.y(),
    });
    // Track somes points
    viewer.addTrackedPoint(model.get().position("trunk", "origin"), Leph::ModelViewer::Green);
    viewer.addTrackedPoint(model.get().centerOfMass("origin"), Leph::ModelViewer::Blue);
    viewer.addTrackedPoint(zmpDoubleLeft, Leph::ModelViewer::Red);
    // Display model
    Leph::ModelDraw(model.get(), viewer);
    // Waiting
    scheduling.wait();
    t += 0.01;
  }
  plot.plot("t", "zmpSingleLeft")
      .plot("t", "zmpSingleRight")
      .plot("t", "zmpDoubleLeft")
      .plot("t", "zmpDoubleRight")
      .plot("t", "pos")
      .render();
  plot.plot("t", "all").render();

  return 0;
}
