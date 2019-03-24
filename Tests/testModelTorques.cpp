#include <iostream>
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Model/NamesModel.h"

int main()
{
  // Model Initialization
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);

  // Inverse kinematics target
  bool isSuccess;
  Eigen::Vector3d trunkPos(0.1, -0.05, 0.20);
  Eigen::Vector3d trunkAxis(0.0, 0.0, -0.3);
  Eigen::Vector3d footPos(0.0, -0.10, 0.0);
  Eigen::Vector3d footAxis(0.0, 0.0, -0.6);

  Leph::ModelViewer viewer(1200, 900);
  double freq = 50.0;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(freq);
  Leph::Plot plot;
  double t = 0.0;
  while (viewer.update())
  {
    // Display model
    Leph::ModelDraw(model.get(), viewer);
    // Waiting
    scheduling.wait();

    // Update inverse kinematics
    if (t >= 3.0)
    {
      trunkPos.y() = -0.02 + 0.05 * sin(1.5 * 3.14 * 0.1 * t);
      footAxis.z() = -0.4 + 0.4 * sin(2.0 * 3.14 * 0.1 * t);
    }
    isSuccess = model.trunkFootIK(Leph::HumanoidFixedModel::LeftSupportFoot, trunkPos, Leph::AxisToMatrix(trunkAxis),
                                  footPos, Leph::AxisToMatrix(footAxis));
    if (!isSuccess)
    {
      std::cout << "IK Error" << std::endl;
      return 1;
    }
    model.get().setDOF("base_x", 0.2 * sin(3.0 * 3.14 * t * 0.1));
    model.get().setDOF("base_y", 0.2);
    model.get().setDOF("base_yaw", 1.5 * sin(2.0 * 3.14 * t * 0.1));

    // Left support
    std::cout << "LEFT SUPPORT:" << std::endl;
    Eigen::VectorXd forceLeftSupport(6);
    Eigen::VectorXd forceLeftFoot(6);
    model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
    Eigen::VectorXd tauDoubleLeft = model.get().inverseDynamicsClosedLoop("right_foot_tip", &forceLeftFoot, false);
    // Convert support contact force
    // in left support foot frame
    Eigen::Matrix3d matLeft = model.get().orientation("left_foot_tip", "origin");
    forceLeftSupport(3) = tauDoubleLeft(0);
    forceLeftSupport(4) = tauDoubleLeft(1);
    forceLeftSupport(5) = tauDoubleLeft(2);
    forceLeftSupport(0) = tauDoubleLeft(5);
    forceLeftSupport(1) = tauDoubleLeft(4);
    forceLeftSupport(2) = tauDoubleLeft(3);
    forceLeftSupport.segment(3, 3) = matLeft * forceLeftSupport.segment(3, 3);

    // Right support
    std::cout << "RIGHT SUPPORT:" << std::endl;
    Eigen::VectorXd forceRightSupport(6);
    Eigen::VectorXd forceRightFoot(6);
    model.setSupportFoot(Leph::HumanoidFixedModel::RightSupportFoot);
    Eigen::VectorXd tauDoubleRight = model.get().inverseDynamicsClosedLoop("left_foot_tip", &forceRightFoot, false);
    // Convert support contact force
    // in right support foot frame
    Eigen::Matrix3d matRight = model.get().orientation("right_foot_tip", "origin");
    forceRightSupport(3) = tauDoubleRight(0);
    forceRightSupport(4) = tauDoubleRight(1);
    forceRightSupport(5) = tauDoubleRight(2);
    forceRightSupport(0) = tauDoubleRight(5);
    forceRightSupport(1) = tauDoubleRight(4);
    forceRightSupport(2) = tauDoubleRight(3);
    forceRightSupport.segment(3, 3) = matRight * forceRightSupport.segment(3, 3);

    // Check that double support torques are the same
    // when computed from left and right support (kinematic root)
    for (const std::string& name : Leph::NamesDOF)
    {
      model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
      size_t indexLeft = model.get().getDOFIndex(name);
      model.setSupportFoot(Leph::HumanoidFixedModel::RightSupportFoot);
      size_t indexRight = model.get().getDOFIndex(name);
      if (fabs(tauDoubleLeft(indexLeft) - tauDoubleRight(indexRight)) > 1e-6)
      {
        std::cout << "ERROR mismatch TORQUE name=" << name << std::endl;
        std::cout << name << " "
                  << "Left:" << indexLeft << ": " << tauDoubleLeft(indexLeft) << " Right:" << indexRight << ": "
                  << tauDoubleRight(indexRight) << std::endl;
        exit(1);
      }
    }

    // Check that contact forces computed from support foot
    // and from flying foot with the other
    // support foot are the same
    if ((forceLeftSupport - forceRightFoot).norm() > 1e-5)
    {
      std::cout << "ERROR mismatch FORCE left foot: " << std::endl;
      std::cout << forceLeftSupport.transpose() << std::endl;
      std::cout << forceRightFoot.transpose() << std::endl;
      exit(1);
    }
    if ((forceRightSupport - forceLeftFoot).norm() > 1e-5)
    {
      std::cout << "ERROR mismatch FORCE right foot: " << std::endl;
      std::cout << forceRightSupport.transpose() << std::endl;
      std::cout << forceLeftFoot.transpose() << std::endl;
      exit(1);
    }

    std::cout << "LeftSupport:  " << forceLeftSupport(0) << " " << forceLeftSupport(1) << " " << forceLeftSupport(2)
              << " " << forceLeftSupport(3) << " " << forceLeftSupport(4) << " " << forceLeftSupport(5) << " "
              << std::endl;
    std::cout << "RightFoot:    " << forceRightFoot(0) << " " << forceRightFoot(1) << " " << forceRightFoot(2) << " "
              << forceRightFoot(3) << " " << forceRightFoot(4) << " " << forceRightFoot(5) << " " << std::endl;
    std::cout << "LeftFoot:     " << forceLeftFoot(0) << " " << forceLeftFoot(1) << " " << forceLeftFoot(2) << " "
              << forceLeftFoot(3) << " " << forceLeftFoot(4) << " " << forceLeftFoot(5) << " " << std::endl;
    std::cout << "RightSupport: " << forceRightSupport(0) << " " << forceRightSupport(1) << " " << forceRightSupport(2)
              << " " << forceRightSupport(3) << " " << forceRightSupport(4) << " " << forceRightSupport(5) << " "
              << std::endl;
    t += 0.01;
  }

  return 0;
}
