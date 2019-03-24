#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/Scheduling.hpp"
#include "Utils/Euler.h"

int main()
{
  // Initialize instance
  Leph::HumanoidFixedModel model(Leph::GrosbanModel);
  Leph::ModelViewer viewer(1200, 900);

  double freq = 50.0;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(freq);
  double t = 0.0;
  while (viewer.update())
  {
    t += 0.01;

    // Targets
    Eigen::Vector3d targetPos;
    Eigen::Vector3d targetAngles;
    std::string frame;
    if (t > 24.0)
    {
      break;
    }
    else if (t > 20.0)
    {
      frame = "trunk";
      targetPos = Eigen::Vector3d(0.0, -0.05, -0.2);
      targetAngles = Eigen::Vector3d(0.0, 0.0, 0.5 * sin(2 * t));
    }
    else if (t > 16.0)
    {
      frame = "trunk";
      targetPos = Eigen::Vector3d(0.0, -0.05, -0.2);
      targetAngles = Eigen::Vector3d(0.0, 0.5 * sin(2 * t), 0.0);
    }
    else if (t > 12.0)
    {
      frame = "trunk";
      targetPos = Eigen::Vector3d(0.0, -0.05, -0.2);
      targetAngles = Eigen::Vector3d(0.5 * sin(2 * t), 0.0, 0.0);
    }
    else if (t > 8.0)
    {
      frame = "origin";
      targetPos = Eigen::Vector3d(0.0, -0.05, 0.05);
      targetAngles = Eigen::Vector3d(0.0, 0.0, 0.5 * sin(2 * t));
    }
    else if (t > 4.0)
    {
      frame = "origin";
      targetPos = Eigen::Vector3d(0.0, -0.05, 0.05);
      targetAngles = Eigen::Vector3d(0.0, 0.5 * sin(2 * t), 0.0);
    }
    else
    {
      frame = "origin";
      targetPos = Eigen::Vector3d(0.0, -0.05, 0.05);
      targetAngles = Eigen::Vector3d(0.5 * sin(2 * t), 0.0, 0.0);
    }

    // Add some pitch/roll on support leg
    model.get().setDOF("left_ankle_pitch", 0.5 * sin(t / 2.0));
    model.get().setDOF("left_ankle_roll", 0.5 * sin(t / 3.0));

    // Run inverse kinematics on right legs
    if (!model.get().legIkRight(frame, targetPos, Leph::EulerToMatrix(targetAngles, Leph::EulerYawPitchRoll)))
    {
      std::cout << "InverseKinematics fail" << std::endl;
    }

    // Track moving point
    viewer.addTrackedPoint(model.get().position("right_foot_tip", "origin"));
    // Display model
    Leph::ModelDraw(model.get(), viewer);
    // Waiting
    scheduling.wait();
  }

  t = 0.0;
  while (viewer.update())
  {
    t += 0.01;

    // Targets
    Eigen::Vector3d targetPos;
    Eigen::Vector3d targetAngles;
    std::string frame;

    targetPos = Eigen::Vector3d(0.0, 0.05 + 0.05 * sin(t / 2), -0.15 + 0.05 * sin(t / 3));
    if (t >= 12.0)
    {
      break;
    }
    else if (t >= 8.0)
    {
      frame = "trunk";
      targetAngles = Eigen::Vector3d(0.0, 0.0, 0.5 * sin(2 * t));
    }
    else if (t >= 4.0)
    {
      frame = "trunk";
      targetAngles = Eigen::Vector3d(0.0, 0.5 * sin(2 * t), 0.0);
    }
    else
    {
      frame = "trunk";
      targetAngles = Eigen::Vector3d(0.5 * sin(2 * t), 0.0, 0.0);
    }

    // Run inverse kinematics on left legs
    if (!model.get().legIkLeft(frame, targetPos, Leph::EulerToMatrix(targetAngles, Leph::EulerYawPitchRoll)))
    {
      std::cout << "InverseKinematics fail" << std::endl;
    }

    // Display model
    Leph::ModelDraw(model.get(), viewer);
    // Waiting
    scheduling.wait();
  }

  return 0;
}
