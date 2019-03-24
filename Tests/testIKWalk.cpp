#include <iostream>
#include "IKWalk/IKWalk.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Utils/AxisAngle.h"
#include "Plot/Plot.hpp"

void testIKWalkOdometry()
{
  // Viewer and model
  Leph::ModelViewer viewer(1200, 900);
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  model.updateBase();

  // Initialing IKWalk
  // Mowgly RoboCup 2016 parameters
  Leph::IKWalk::Parameters params;
  params.freq = 1.7;
  params.enabledGain = 1.0;
  params.supportPhaseRatio = 0.0;
  params.footYOffset = 0.025;
  params.stepGain = 0.04;
  params.riseGain = 0.035;
  params.turnGain = 0.0;
  params.lateralGain = 0.0;
  params.trunkZOffset = 0.02;
  params.swingGain = 0.01999999955;
  params.swingRollGain = 0.0;
  params.swingPhase = 0.25;
  params.stepUpVel = 5.0;
  params.stepDownVel = 5.0;
  params.riseUpVel = 5.0;
  params.riseDownVel = 5.0;
  params.swingPause = 0.0;
  params.swingVel = 4.0;
  params.trunkXOffset = 0.009999999775;
  params.trunkYOffset = 0.0;
  params.trunkPitch = 0.1919862181;
  params.trunkRoll = 0.0;
  params.extraLeftX = 0.0;
  params.extraLeftY = 0.0;
  params.extraLeftZ = 0.0;
  params.extraLeftYaw = 0.0;
  params.extraLeftPitch = 0.0;
  params.extraLeftRoll = 0.0;
  params.extraRightX = 0.0;
  params.extraRightY = 0.0;
  params.extraRightZ = 0.0;
  params.extraRightYaw = 0.0;
  params.extraRightPitch = 0.0;
  params.extraRightRoll = 0.0;
  double phase = 0.0;

  Leph::Plot plot;
  Leph::Scheduling scheduling(50.0);
  double t = 0.0;
  while (viewer.update())
  {
    t += 0.01;
    // Run walk generator
    bool success = Leph::IKWalk::walk(model.get(), params, phase, 0.01);
    if (!success)
    {
      std::cout << "IKWalk inverse kinematics failed" << std::endl;
      return;
    }
    // Contraint the model on the ground
    model.updateBase();
    // Track moving points
    viewer.addTrackedPoint(model.get().position("left_foot_tip", "origin"), Leph::ModelViewer::Red);
    viewer.addTrackedPoint(model.get().position("trunk", "origin"), Leph::ModelViewer::Green);
    viewer.addTrackedPoint(model.get().centerOfMass("origin"), Leph::ModelViewer::Blue);
    viewer.addTrackedPoint(model.get().position("right_foot_tip", "origin"), Leph::ModelViewer::Purple);
    // Display model
    Leph::ModelDraw(model.get(), viewer);
    // Waiting
    scheduling.wait();
    // Plot
    plot.add(Leph::VectorLabel("t", t, "phase", phase, "left_foot_x",
                               model.get().position("left_foot_tip", "origin").x(), "trunk_x",
                               model.get().position("trunk", "origin").x()));
  }
  plot.plot("t", "all").render();
}

void testStaticPose()
{
  std::cout << "Mowgly 2016 RoboCup static walk double support pose" << std::endl;

  // Viewer and model
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  model.updateBase();

  // Initialing IKWalk
  // Mowgly RoboCup 2016 parameters
  Leph::IKWalk::Parameters params;
  params.freq = 1.7;
  params.enabledGain = 1.0;
  params.supportPhaseRatio = 0.0;
  params.footYOffset = 0.025;
  params.stepGain = 0.0;
  params.riseGain = 0.035;
  params.turnGain = 0.0;
  params.lateralGain = 0.0;
  params.trunkZOffset = 0.02;
  params.swingGain = 0.01999999955;
  params.swingRollGain = 0.0;
  params.swingPhase = 0.25;
  params.stepUpVel = 5.0;
  params.stepDownVel = 5.0;
  params.riseUpVel = 5.0;
  params.riseDownVel = 5.0;
  params.swingPause = 0.0;
  params.swingVel = 4.0;
  params.trunkXOffset = 0.009999999775;
  params.trunkYOffset = 0.0;
  params.trunkPitch = 0.1919862181;
  params.trunkRoll = 0.0;
  params.extraLeftX = 0.0;
  params.extraLeftY = 0.0;
  params.extraLeftZ = 0.0;
  params.extraLeftYaw = 0.0;
  params.extraLeftPitch = 0.0;
  params.extraLeftRoll = 0.0;
  params.extraRightX = 0.0;
  params.extraRightY = 0.0;
  params.extraRightZ = 0.0;
  params.extraRightYaw = 0.0;
  params.extraRightPitch = 0.0;
  params.extraRightRoll = 0.0;
  // Run walk generator
  double phase = 0.0;
  bool success = Leph::IKWalk::walk(model.get(), params, phase, 0.00);
  if (!success)
  {
    std::cout << "IKWalk inverse kinematics failed" << std::endl;
    return;
  }
  // Contraint the model on the ground
  model.updateBase();

  // Display pose state
  std::cout << "TrunkPos:  " << model.get().position("trunk", "left_foot_tip").transpose() << std::endl;
  std::cout << "TrunkAxis: "
            << Leph::MatrixToAxis(model.get().orientation("trunk", "left_foot_tip").transpose()).transpose()
            << std::endl;
  std::cout << "FootPos:  " << model.get().position("right_foot_tip", "left_foot_tip").transpose() << std::endl;
  std::cout << "FootAxis: "
            << Leph::MatrixToAxis(model.get().orientation("right_foot_tip", "left_foot_tip").transpose()).transpose()
            << std::endl;
}

int main()
{
  // Viewer and model
  Leph::ModelViewer viewer(1200, 900);
  Leph::HumanoidFloatingModel modelOld(Leph::SigmabanModel);
  modelOld.putOnGround();
  modelOld.setStatePosY(0.20);
  Leph::HumanoidFixedModel modelNew(Leph::SigmabanModel);
  modelNew.updateBase();

  // Initialing CartWalk generator
  Leph::CartWalkProxy walk;
  Leph::VectorLabel paramsOld = walk.buildParams();
  paramsOld("dynamic:enabled") = 1;
  paramsOld("static:timeGain") = 0.5;
  paramsOld("dynamic:step") = 0.0;
  paramsOld("dynamic:turn") = 10.0;
  paramsOld("dynamic:step") = 4.0;
  paramsOld("static:riseGain") = 3.0;
  paramsOld("static:swingPhase") = 0.0;
  paramsOld("static:swingGain") = 3.0;
  paramsOld("static:zOffset") = 4.0;
  paramsOld("static:yLat") = 0.0;
  paramsOld("static:yOffset") = 0.0;

  // Initialing IKWalk
  Leph::IKWalk::Parameters params;
  params.freq = 0.3;
  params.enabledGain = 1.0;
  params.supportPhaseRatio = 0.6;
  params.footYOffset = 0.01;
  params.stepGain = 0.08;
  params.riseGain = 0.06;
  params.turnGain = 0.0;
  params.lateralGain = 0.0;
  params.trunkZOffset = 0.05;
  params.swingGain = 0.02;
  params.swingRollGain = 0.0;
  params.swingPhase = 0.0;
  params.stepUpVel = 2.0;
  params.stepDownVel = 2.0;
  params.riseUpVel = 2.0;
  params.riseDownVel = 2.0;
  params.swingPause = 0.1;
  params.swingVel = 5.0;
  params.trunkXOffset = 0.0;
  params.trunkYOffset = 0.0;
  params.trunkPitch = 0.5;
  params.trunkRoll = -0.2;
  params.extraLeftX = 0.0;
  params.extraLeftY = 0.0;
  params.extraLeftZ = 0.0;
  params.extraLeftYaw = 0.0;
  params.extraLeftPitch = 0.0;
  params.extraLeftRoll = 0.0;
  params.extraRightX = 0.0;
  params.extraRightY = 0.0;
  params.extraRightZ = 0.0;
  params.extraRightYaw = 0.0;
  params.extraRightPitch = 0.0;
  params.extraRightRoll = 0.0;
  double phase = 0.0;

  Leph::Scheduling scheduling(50.0);
  double t = 0.0;
  while (viewer.update())
  {
    t += 0.01;
    // CartWalk generator
    walk.exec(0.01, paramsOld);
    // Adapt CartWalk convention to Model convention
    Leph::VectorLabel output = walk.lastOutputs().rename("output", "");
    output("left_hip_roll") *= -1;
    output("left_ankle_pitch") *= -1;
    output("right_hip_pitch") *= -1;
    output("right_knee") *= -1;
    output("right_hip_roll") *= -1;
    // Convertion to radian
    output.mulOp(M_PI / 180.0);
    // Send motor output to model
    modelOld.setDOF(output);
    // Contraint the model on the ground
    modelOld.putOnGround();
    // Track moving points
    viewer.addTrackedPoint(modelOld.position("right_foot_tip", "origin"), Leph::ModelViewer::Yellow);
    viewer.addTrackedPoint(modelOld.centerOfMass("origin"), Leph::ModelViewer::Cyan);
    // Display model
    Leph::ModelDraw(modelOld, viewer);

    // Add some trunk orientation oscillation
    params.trunkPitch = 0.2 * sin(t * 2.0);
    params.trunkRoll = 0.1 * sin(t * 3.0);
    // Run walk generator
    bool success = Leph::IKWalk::walk(modelNew.get(), params, phase, 0.01);
    if (!success)
    {
      std::cout << "IKWalk inverse kinematics failed" << std::endl;
      return -1;
    }
    // Contraint the model on the ground
    modelNew.updateBase();
    // Track moving points
    viewer.addTrackedPoint(modelNew.get().position("left_foot_tip", "origin"), Leph::ModelViewer::Red);
    viewer.addTrackedPoint(modelNew.get().position("trunk", "origin"), Leph::ModelViewer::Green);
    viewer.addTrackedPoint(modelNew.get().centerOfMass("origin"), Leph::ModelViewer::Blue);
    viewer.addTrackedPoint(modelNew.get().position("right_foot_tip", "origin"), Leph::ModelViewer::Purple);
    // Display model
    Leph::ModelDraw(modelNew.get(), viewer);
    // Waiting
    scheduling.wait();
    // Display parameters
    Leph::VectorLabel vectParams;
    Leph::IKWalk::convertParameters(vectParams, params);
    std::cout << vectParams << std::endl;
  }

  // Test IKWalk Odometry and paramater meanings
  testIKWalkOdometry();

  // Show trunk-foot IK pose
  testStaticPose();

  return 0;
}
