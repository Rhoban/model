#include <iostream>
#include "Model/HumanoidSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/AxisAngle.h"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

/**
 * Test Humanoid foot contacts
 */
void testContacts()
{
  Leph::HumanoidSimulation sim(Leph::SigmabanModel);
  Leph::ModelViewer viewer(1200, 900);
  Leph::HumanoidFixedModel goalModel(Leph::SigmabanModel);

  Eigen::Vector3d trunkPos(-0.00557785331559037, -0.0115849568418458, 0.28);
  // Eigen::Vector3d trunkAxis(-0.672036398746933, 0.0743358280850477, 0.0028323027017884);
  Eigen::Vector3d trunkAxis(0.0, 0.0, 0.0);
  Eigen::Vector3d footPos(0.0, -0.12, 0.02);
  // Eigen::Vector3d footPos(0.0208647084129351, -0.095, 0.0591693358237435);
  // trunkPos.y() += 0.04;

  bool success =
      goalModel.trunkFootIK(Leph::HumanoidFixedModel::LeftSupportFoot, trunkPos, Leph::AxisToMatrix(trunkAxis), footPos,
                            Leph::AxisToMatrix(Eigen::Vector3d::Zero()));
  if (!success)
  {
    std::cout << "IK ERROR" << std::endl;
    exit(1);
  }
  for (const std::string& name : Leph::NamesDOF)
  {
    sim.setPos(name, goalModel.get().getDOF(name));
    sim.setGoal(name, goalModel.get().getDOF(name));
  }

  sim.putOnGround();
  sim.putFootAt(0.0, 0.0);

  Leph::Plot plot;
  double t = 0.0;
  size_t count = 0;
  while (viewer.update())
  {
    count++;
    sim.setGoal("left_ankle_roll", goalModel.get().getDOF("left_ankle_roll") - 0.4 * sin(2.0 * 3.14 * t * 0.05));
    std::cout << "********** " << count << " *********" << std::endl;
    sim.update(0.001);
    t += 0.001;
    sim.printCleatsStatus();
    sim.printCleatsStatus(plot);
    Leph::ModelDraw(sim.model(), viewer);
    Leph::CleatsDraw(sim, viewer);
    Eigen::Vector3d com = sim.model().centerOfMass("origin");
    com.z() = 0.0;
    viewer.addTrackedPoint(com, Leph::ModelViewer::Red);
    // if (count >= 450) break;
    // if (count >= 208) break;
    // if (count >= 3810) break;
  }
  plot.plot("index", "left_cleat_1:force")
      .plot("index", "left_cleat_2:force")
      .plot("index", "left_cleat_3:force")
      .plot("index", "left_cleat_4:force")
      .plot("index", "left_cleat_1:isActive")
      .plot("index", "left_cleat_2:isActive")
      .plot("index", "left_cleat_3:isActive")
      .plot("index", "left_cleat_4:isActive")
      .render();
  plot.plot("index", "right_cleat_1:force")
      .plot("index", "right_cleat_2:force")
      .plot("index", "right_cleat_3:force")
      .plot("index", "right_cleat_4:force")
      .render();
}

/**
 * Test IKWalk simulation
 */
void testWalk()
{
  Leph::HumanoidSimulation sim(Leph::SigmabanModel);
  Leph::HumanoidFloatingModel goalModel(Leph::SigmabanModel);
  Leph::ModelViewer viewer(1200, 900);

  // Initialing IKWalk
  Leph::IKWalk::Parameters params;
  // Mowgly working 2016 parameters
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

  double phase = 0.0;
  bool isFirstIteration = true;
  while (viewer.update())
  {
    // Run walk generator
    Leph::IKWalk::walk(goalModel, params, phase, 0.01);
    // Import target position
    for (const std::string& name : Leph::NamesDOF)
    {
      sim.setGoal(name, goalModel.getDOF(name));
    }
    // On first iteration, assign pos/vel state
    if (isFirstIteration)
    {
      for (const std::string& name : Leph::NamesDOF)
      {
        sim.setPos(name, goalModel.getDOF(name));
        sim.setGoal(name, goalModel.getDOF(name));
        sim.setVel(name, 0.0);
      }
      sim.putOnGround();
      sim.putFootAt(0.0, 0.0, Leph::HumanoidFixedModel::LeftSupportFoot);
      isFirstIteration = false;
    }
    // Run simulation
    for (int k = 0; k < 100; k++)
    {
      sim.update(0.0001);
    }
    // Display
    Leph::ModelDraw(sim.model(), viewer);
    Leph::CleatsDraw(sim, viewer);
  }
}

int main()
{
  testContacts();
  testWalk();

  return 0;
}
