#include <iostream>
#include <string>
#include <stdexcept>

#include <lwpr_eigen.hpp>
#include "Model/HumanoidFixedModel.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Utils/Angle.h"

void computeDelta(double walkStepX, double walkStepY, double walkStepTheta, bool isLeftSupportFoot, double& deltaX,
                  double& deltaY, double& deltaTheta)
{
  // Current support foot
  Leph::HumanoidFixedModel::SupportFoot supportFoot =
      (isLeftSupportFoot ? Leph::HumanoidFixedModel::LeftSupportFoot : Leph::HumanoidFixedModel::RightSupportFoot);

  // Model initialization
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  model.setSupportFoot(supportFoot);
  // IKWalk initialization
  double phase = (isLeftSupportFoot ? 0.0001 : 0.5001);
  Leph::IKWalk::Parameters paramsWalk;
  paramsWalk.freq = 1.7;
  paramsWalk.enabledGain = 1.0;
  paramsWalk.supportPhaseRatio = 0.0;
  paramsWalk.footYOffset = 0.025;
  paramsWalk.stepGain = walkStepX;
  paramsWalk.riseGain = 0.035;
  paramsWalk.turnGain = walkStepTheta;
  paramsWalk.lateralGain = walkStepY;
  paramsWalk.swingGain = 0.02;
  paramsWalk.swingRollGain = 0.0;
  paramsWalk.swingPhase = 0.25;
  paramsWalk.stepUpVel = 4.0;
  paramsWalk.stepDownVel = 4.0;
  paramsWalk.riseUpVel = 4.0;
  paramsWalk.riseDownVel = 4.0;
  paramsWalk.swingPause = 0.0;
  paramsWalk.swingVel = 4.0;
  paramsWalk.trunkPitch = 0.15;
  paramsWalk.trunkRoll = 0.0;
  paramsWalk.extraLeftX = 0.0;
  paramsWalk.extraLeftY = 0.0;
  paramsWalk.extraLeftZ = 0.0;
  paramsWalk.extraLeftYaw = 0.0;
  paramsWalk.extraLeftPitch = 0.0;
  paramsWalk.extraLeftRoll = 0.0;
  paramsWalk.extraRightX = 0.0;
  paramsWalk.extraRightY = 0.0;
  paramsWalk.extraRightZ = 0.0;
  paramsWalk.extraRightYaw = 0.0;
  paramsWalk.extraRightPitch = 0.0;
  paramsWalk.extraRightRoll = 0.0;
  paramsWalk.trunkXOffset = 0.02;
  paramsWalk.trunkYOffset = 0.0;
  paramsWalk.trunkZOffset = 0.02;

  // Compute IKWalk generation
  bool isSuccess = Leph::IKWalk::walk(model.get(), paramsWalk, phase, 0.02);
  if (!isSuccess)
  {
    throw std::logic_error("IKWalk error");
  }
  model.updateBase();

  // Compute initial head position
  Eigen::Vector3d position1 = model.get().position("camera", "origin");
  double theta2 = model.get().orientationYaw("camera", "origin");

  // Run one step
  while (model.getSupportFoot() == supportFoot)
  {
    bool isSuccess = Leph::IKWalk::walk(model.get(), paramsWalk, phase, 0.02);
    if (!isSuccess)
    {
      throw std::logic_error("IKWalk error");
    }
    model.updateBase();
  }

  // Compute final head position
  Eigen::Vector3d position2 = model.get().position("camera", "origin");
  double theta1 = model.get().orientationYaw("camera", "origin");

  // Compute delta head pose in robot's frame
  double x1 = position2.x();
  double x2 = position1.x();
  double y1 = position2.y();
  double y2 = position1.y();
  deltaTheta = Leph::AngleDistance(theta2, theta1);
  double tmpX = x1 - x2;
  double tmpY = y1 - y2;
  deltaX = cos(-theta2) * tmpX - sin(-theta2) * tmpY;
  deltaY = sin(-theta2) * tmpX + cos(-theta2) * tmpY;
}

void computeOdometry(double walkStepX1, double walkStepY1, double walkStepTheta1, double walkStepX2, double walkStepY2,
                     double walkStepTheta2, bool isLeftSupportFoot, LWPR_Object* modelX, LWPR_Object* modelY,
                     LWPR_Object* modelTheta, double& deltaX, double& deltaY, double& deltaTheta)
{
  // Compute the two last delta
  double deltaX1 = 0.0;
  double deltaY1 = 0.0;
  double deltaTheta1 = 0.0;
  double deltaX2 = 0.0;
  double deltaY2 = 0.0;
  double deltaTheta2 = 0.0;
  computeDelta(walkStepX1, walkStepY1, walkStepTheta1, isLeftSupportFoot, deltaX1, deltaY1, deltaTheta1);
  computeDelta(walkStepX2, walkStepY2, walkStepTheta2, !isLeftSupportFoot, deltaX2, deltaY2, deltaTheta2);

  // Compute displacement correction
  Eigen::VectorXd in(7);
  in << deltaX1, deltaY1, deltaTheta1, deltaX2, deltaY2, deltaTheta2, (int)isLeftSupportFoot;
  deltaX = modelX->predict(in, 0.0)(0);
  deltaY = modelY->predict(in, 0.0)(0);
  deltaTheta = modelTheta->predict(in, 0.0)(0);
}

void odometryIntegration(double deltaX, double deltaY, double deltaTheta, double& poseX, double& poseY,
                         double& poseTheta)
{
  double tmpX = deltaX * cos(poseTheta) - deltaY * sin(poseTheta);
  double tmpY = deltaX * sin(poseTheta) + deltaY * cos(poseTheta);
  poseX += tmpX;
  poseY += tmpY;
  poseTheta += deltaTheta;
}

int main()
{
  // Load and initialized learned odometry model
  std::string filepathX = "../Data/Odometry/model_walk_delta_x.bin";
  std::string filepathY = "../Data/Odometry/model_walk_delta_y.bin";
  std::string filepathTheta = "../Data/Odometry/model_walk_delta_theta.bin";
  LWPR_Object* modelX = new LWPR_Object(filepathX.c_str());
  LWPR_Object* modelY = new LWPR_Object(filepathY.c_str());
  LWPR_Object* modelTheta = new LWPR_Object(filepathTheta.c_str());

  // Walk orders
  double walkStepX = 0.02;
  double walkStepY = 0.01;
  double walkStepTheta = 0.1;

  // Computed head displacement delta
  double deltaX = 0.0;
  double deltaY = 0.0;
  double deltaTheta = 0.0;

  // Integrated robot pose
  double poseX = 0.0;
  double poseY = 0.0;
  double poseTheta = 0.0;

  bool isLeftSupportFoot = true;

  for (size_t k = 0; k < 10; k++)
  {
    // Compute corrected displacement delta
    // with given last two walk orders and
    // current support foot
    computeOdometry(walkStepX, walkStepY, walkStepTheta, walkStepX, walkStepY, walkStepTheta, isLeftSupportFoot, modelX,
                    modelY, modelTheta, deltaX, deltaY, deltaTheta);
    // Integrate delta displacements
    odometryIntegration(deltaX, deltaY, deltaTheta, poseX, poseY, poseTheta);
    std::cout << k << " " << deltaX << " " << deltaY << " " << deltaTheta << poseX << " " << poseY << " " << poseTheta
              << std::endl;
    // Swap support foot
    isLeftSupportFoot = !isLeftSupportFoot;
  }

  // Free LWPR model
  delete modelX;
  delete modelY;
  delete modelTheta;

  return 0;
}
