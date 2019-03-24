#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"

int main()
{
  Leph::HumanoidFloatingModel model(Leph::SigmabanModel);
  model.putOnGround();

  Leph::ModelViewer viewer(1200, 900);

  // Inverse Kinematics
  Leph::InverseKinematics inv(model);
  // Declare model degrees of freedom
  inv.addDOF("right_hip_pitch");
  inv.addDOF("right_hip_roll");
  inv.addDOF("right_knee");
  inv.addDOF("right_ankle_pitch");
  inv.addDOF("right_ankle_roll");
  inv.addDOF("left_hip_pitch");
  inv.addDOF("left_hip_roll");
  inv.addDOF("left_knee");
  inv.addDOF("left_ankle_pitch");
  inv.addDOF("left_ankle_roll");
  inv.addDOF("base_x");
  inv.addDOF("base_y");
  inv.addDOF("base_z");
  // Declare degree of freefom box bounds
  // XXX Not fully implemented
  inv.setLowerBound("left_knee", 0.0);
  inv.setLowerBound("right_knee", 0.0);

  // Declare target position
  inv.addTargetPosition("flying_foot", "right_foot_tip");
  inv.addTargetPosition("support_foot", "left_foot_tip");
  // target of center of mass
  inv.addTargetCOM();
  inv.targetCOM().z() -= 0.05;
  // Target orientation
  inv.addTargetOrientation("flying_foot", "right_foot_tip");
  inv.addTargetOrientation("support_foot", "left_foot_tip");

  Leph::Chrono chrono;
  double t = 0.0;
  while (viewer.update())
  {
    t += 0.01;

    // Update targets
    inv.targetPosition("flying_foot").z() = 0.05 + 0.02 * sin(t);
    inv.targetPosition("flying_foot").x() = 0.05 + 0.02 * sin(2.0 * t);
    inv.targetCOM().y() = 0.01 * sin(t);
    inv.targetCOM().x() = 0.01 * sin(2.0 * t);

    chrono.start("InverseKinematics");
    // Compute Inverse Kinematics
    inv.run(0.0001, 100);
    chrono.stop("InverseKinematics");
    chrono.print();
    std::cout << "ERRORS" << std::endl;
    std::cout << "COM pos: " << inv.errorCOM() << std::endl;
    std::cout << "Flying foot pos: " << inv.errorPosition("flying_foot") << std::endl;
    std::cout << "Support foot pos: " << inv.errorPosition("support_foot") << std::endl;
    std::cout << "Flying foot orientation: " << inv.errorOrientation("flying_foot") << std::endl;
    std::cout << "Support foot orientation: " << inv.errorOrientation("support_foot") << std::endl;

    // Display
    Eigen::Vector3d pt = model.centerOfMass("origin");
    viewer.addTrackedPoint(pt);
    Leph::ModelDraw(model, viewer);
  }

  return 0;
}
