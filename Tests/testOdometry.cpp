#include <iostream>
#include "Plot/Plot.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Odometry/Odometry.hpp"

int main()
{
  // HumanoidFixedModel
  Leph::HumanoidFixedModel model(Leph::SigmabanModel);
  model.updateBase();

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
  params.trunkPitch = 0.2;
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

  // Odometry
  Leph::Odometry odometry(Leph::OdometryDisplacementModel::DisplacementLinearSimpleXY,
                          Leph::OdometryNoiseModel::NoiseDisable);
  Eigen::VectorXd tmpParams = odometry.getParameters();
  tmpParams(1) = 1.5;
  tmpParams(6) = 1.2;
  odometry.setParameters(tmpParams);

  Leph::Plot plot;
  double phase = 0.0;
  for (double t = 0.0; t < 30.0; t += 0.01)
  {
    // Run walk generator
    bool success = Leph::IKWalk::walk(model.get(), params, phase, 0.01);
    if (!success)
    {
      std::cout << "IKWalk inverse kinematics failed t=" << t << std::endl;
      return -1;
    }
    model.updateBase();
    // Odometry
    odometry.update(model, nullptr);
    // Plot
    plot.add(Leph::VectorLabel("t", t, "x", model.get().getPose().x(), "y", model.get().getPose().y(), "corrected_x",
                               odometry.state().x(), "corrected_y", odometry.state().y()));
    if (t >= 20.0 && t < 20.02)
    {
      params.turnGain = -0.2;
      model.get().setDOF("base_y", -0.2);
      model.get().setDOF("base_yaw", -M_PI / 2.0);
      odometry.reset();
    }
  }
  plot.plot("x", "y", Leph::Plot::LinesPoints, "t")
      .plot("corrected_x", "corrected_y", Leph::Plot::LinesPoints, "t")
      .render();

  return 0.0;
}
