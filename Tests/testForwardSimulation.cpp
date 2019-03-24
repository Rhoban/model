#include <iostream>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/RBDLRootUpdate.h"
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/AxisAngle.h"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

/**
 * Simple pendulum model. No contact. No floating base. 3 DOFs.
 */
void testForward()
{
  // Simple pendulum
  Leph::Model model("../Data/pendulum_triple.urdf");
  model.setDOF("roll1", M_PI / 2.0);
  model.setDOF("roll2", -0.8);

  // Simulator
  Leph::ForwardSimulation sim(model);

  Leph::ModelViewer viewer(1200, 900);
  Leph::Plot plot;
  double t = 0.0;
  size_t count = 0;
  while (viewer.update())
  {
    for (int k = 0; k < 10; k++)
    {
      t += 0.0001;
      std::cout << "********** count=" << count << std::endl;
      sim.update(0.0001);
      count++;
      Leph::VectorLabel vect;
      for (size_t i = 0; i < model.sizeDOF(); i++)
      {
        vect.append("backlash_pos:" + model.getDOFName(i), sim.jointModel(i).getBacklashStatePos());
        vect.append("backlash_enabled:" + model.getDOFName(i), sim.jointModel(i).getBacklashStateEnabled());
        vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
        vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
        vect.append("joint:" + model.getDOFName(i), sim.jointTorques()(i));
        vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
        vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
        vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
        vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
      }
      plot.add(vect);
    }
    model.setDOFVect(sim.positions());
    Leph::ModelDraw(model, viewer);
  }
  plot.plot("index", "backlash_pos:*").plot("index", "backlash_enabled:*").render();
  plot.plot("index", "acc:*").render();
  plot.plot("index", "joint:*").render();
  plot.plot("index", "joint:*").plot("index", "friction:*").plot("index", "control:*").render();
  plot.plot("index", "pos:*").render();
  plot.plot("index", "vel:*").render();
  plot.plot("index", "goal:*").render();
}

/**
 * Simple pendulum model. Floating base and
 * full contact constraints. The floating base
 * is not at the same body as constraints.
 */
void testConstraints()
{
  // Load model from URDF file
  std::string urdfFile = "../Data/pendulum_triple.urdf";
  Leph::RBDL::Model modelOld;
  if (!Leph::RBDL::Addons::URDFReadFromFile(urdfFile.c_str(), &modelOld, false))
  {
    return;
  }

  // Select new RBDL body id root
  // Select the last body of the triple pendulum
  size_t frameRootId = 3;
  // Update old urdf model with new root frame
  Leph::RBDL::Model modelNew = Leph::RBDLRootUpdate(modelOld, frameRootId, true);
  // Initialize base model
  // Simple pendulum
  Leph::Model model(modelNew);
  model.setDOF("roll1", M_PI / 2.0);
  model.setDOF("roll2", -0.8);

  // Floating base transformation to put
  // the pendulum base on the ground at the origin
  //(copy paste from HumanoidSimulation)
  std::string foot;
  Eigen::Vector3d posLeft = model.position("roll3", "base2");
  Eigen::Vector3d posRight = model.position("roll3", "base2");
  if (posLeft.z() >= posRight.z())
  {
    foot = "base2";
  }
  else
  {
    foot = "base2";
  }
  Eigen::Matrix3d rotOriginToTrunk = model.orientation("roll3", "origin");
  Eigen::Matrix3d rotOriginToFoot = model.orientation(foot, "origin");
  Eigen::Matrix3d rotation = rotOriginToTrunk * rotOriginToFoot.transpose();
  Eigen::Vector3d angles;
  angles(0) = atan2(rotation(1, 2), rotation(2, 2));
  angles(1) = atan2(-rotation(0, 2), sqrt(rotation(0, 0) * rotation(0, 0) + rotation(0, 1) * rotation(0, 1)));
  angles(2) = atan2(rotation(0, 1), rotation(0, 0));
  model.setDOF("base_roll", angles(0));
  model.setDOF("base_pitch", angles(1));
  Eigen::VectorXd pos = model.position(foot, "origin");
  double height = model.getDOF("base_z");
  model.setDOF("base_z", height - pos.z());
  std::string foot2;
  Eigen::Vector3d posLeft2 = model.position("roll3", "base2");
  Eigen::Vector3d posRight2 = model.position("roll3", "base2");
  if (posLeft2.z() >= posRight2.z())
  {
    foot2 = "base2";
  }
  else
  {
    foot2 = "base2";
  }
  Eigen::Vector3d posFoot = model.position(foot2, "origin");
  model.setDOF("base_x", model.getDOF("base_x") - posFoot.x());
  model.setDOF("base_y", model.getDOF("base_y") - posFoot.y());

  // Full 6 DOFs constraints on the ground
  Leph::RBDL::ConstraintSet constraints;
  constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
  // Z 0
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 1
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Y 2
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));
  // Z 3
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.1, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 4
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.1, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Z 5
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.1, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  constraints.Bind(model.getRBDLModel());

  // Simulator
  Leph::ForwardSimulation sim(model);

  Leph::ModelViewer viewer(1200, 900);
  Leph::Plot plot;
  double t = 0.0;
  size_t count = 0;
  while (viewer.update())
  {
    for (int k = 0; k < 10; k++)
    {
      t += 0.0001;
      std::cout << "********** count=" << count << std::endl;
      sim.update(0.0001, &constraints);
      count++;
      Leph::VectorLabel vect;
      for (size_t i = 0; i < model.sizeDOF(); i++)
      {
        vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
        vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
        vect.append("joint:" + model.getDOFName(i), sim.jointTorques()(i));
        vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
        vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
        vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
        vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
      }
      vect.append("force:1", constraints.force(0));
      vect.append("force:2", constraints.force(3));
      vect.append("force:3", constraints.force(5));
      plot.add(vect);
    }
    model.setDOFVect(sim.positions());
    Leph::ModelDraw(model, viewer);
  }
  plot.plot("index", "force:*").render();
  plot.plot("index", "acc:*").render();
  plot.plot("index", "joint:*").render();
  plot.plot("index", "joint:*").plot("index", "friction:*").plot("index", "control:*").render();
  plot.plot("index", "pos:*").render();
  plot.plot("index", "vel:*").render();
  plot.plot("index", "goal:*").render();
}

/**
 * Simple pendulum model. Fixed base and
 * full contact constraints at pendulum tip.
 */
void testFullConstraints()
{
  // Load model from URDF file
  std::string urdfFile = "../Data/pendulum_triple.urdf";
  Leph::RBDL::Model modelOld;
  if (!Leph::RBDL::Addons::URDFReadFromFile(urdfFile.c_str(), &modelOld, false))
  {
    return;
  }

  // Select new RBDL body id root
  // Select the last body of the triple pendulum
  size_t frameRootId = 0;
  // Update old urdf model with new root frame
  Leph::RBDL::Model modelNew = Leph::RBDLRootUpdate(modelOld, frameRootId, true);
  // Initialize base model
  // Simple pendulum
  Leph::Model model(modelNew);
  model.setDOF("roll1", M_PI / 2.0);
  model.setDOF("roll2", -0.8);

  // Full 6 DOFs constraints on the ground
  Leph::RBDL::ConstraintSet constraints;
  constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
  // Z 0
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("tip")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 1
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("tip")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Y 2
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("tip")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));
  // Z 0
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 1
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Y 2
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("base2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));

  constraints.Bind(model.getRBDLModel());

  // Simulator
  Leph::ForwardSimulation sim(model);

  Leph::ModelViewer viewer(1200, 900);
  Leph::Plot plot;
  double t = 0.0;
  size_t count = 0;
  while (viewer.update())
  {
    for (int k = 0; k < 10; k++)
    {
      t += 0.0001;
      std::cout << "********** count=" << count << std::endl;
      sim.update(0.0001, &constraints);
      count++;
      Leph::VectorLabel vect;
      for (size_t i = 0; i < model.sizeDOF(); i++)
      {
        vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
        vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
        vect.append("joint:" + model.getDOFName(i), sim.jointTorques()(i));
        vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
        vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
        vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
        vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
      }
      plot.add(vect);
    }
    if (count >= 2070)
      break;
    model.setDOFVect(sim.positions());
    Leph::ModelDraw(model, viewer);
  }
  plot.plot("index", "acc:*").render();
  plot.plot("index", "joint:*").render();
  plot.plot("index", "joint:*").plot("index", "friction:*").plot("index", "control:*").render();
  plot.plot("index", "pos:*").render();
  plot.plot("index", "vel:*").render();
  plot.plot("index", "goal:*").render();
}

/**
 * Full Sigmaban model. Floating base on the trunk and
 * 6 DOFs contact constraints on left foot.
 */
void testHumanoid()
{
  // Full humanoid
  Leph::HumanoidModel model(Leph::SigmabanModel, "trunk", true);

  // Simulator
  Leph::ForwardSimulation sim(model);

  // Single support static position
  Eigen::Vector3d trunkPos(-0.00557785331559037, -0.0115849568418458, 0.28);
  Eigen::Vector3d trunkAxis(-0.672036398746933, 0.0743358280850477, 0.0028323027017884);
  Eigen::Vector3d footPos(0.0208647084129351, -0.095, 0.0591693358237435);
  // Forward offset
  trunkPos.x() += 0.08;
  Leph::HumanoidFixedModel goalModel(Leph::SigmabanModel);
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
    size_t index = model.getDOFIndex(name);
    sim.positions()(index) = goalModel.get().getDOF(name);
    sim.goals()(index) = goalModel.get().getDOF(name);
    model.setDOF(index, goalModel.get().getDOF(name));
  }

  // Update the floating base to set the support foot
  // flat at origin
  std::string foot;
  Eigen::Vector3d posLeft = model.position("trunk", "left_foot_tip");
  Eigen::Vector3d posRight = model.position("trunk", "right_foot_tip");
  if (posLeft.z() >= posRight.z())
  {
    foot = "left_foot_tip";
  }
  else
  {
    foot = "right_foot_tip";
  }
  Eigen::Matrix3d rotOriginToTrunk = model.orientation("trunk", "origin");
  Eigen::Matrix3d rotOriginToFoot = model.orientation(foot, "origin");
  Eigen::Matrix3d rotation = rotOriginToTrunk * rotOriginToFoot.transpose();
  Eigen::Vector3d angles;
  angles(0) = atan2(rotation(1, 2), rotation(2, 2));
  angles(1) = atan2(-rotation(0, 2), sqrt(rotation(0, 0) * rotation(0, 0) + rotation(0, 1) * rotation(0, 1)));
  angles(2) = atan2(rotation(0, 1), rotation(0, 0));
  model.setDOF("base_roll", angles(0));
  model.setDOF("base_pitch", angles(1));
  Eigen::VectorXd pos = model.position(foot, "origin");
  double height = model.getDOF("base_z");
  model.setDOF("base_z", height - pos.z());
  std::string foot2;
  Eigen::Vector3d posLeft2 = model.position("trunk", "left_foot_tip");
  Eigen::Vector3d posRight2 = model.position("trunk", "right_foot_tip");
  if (posLeft2.z() >= posRight2.z())
  {
    foot2 = "left_foot_tip";
  }
  else
  {
    foot2 = "right_foot_tip";
  }
  Eigen::Vector3d posFoot = model.position(foot2, "origin");
  model.setDOF("base_x", model.getDOF("base_x") - posFoot.x());
  model.setDOF("base_y", model.getDOF("base_y") - posFoot.y());
  for (const std::string& name : Leph::NamesBase)
  {
    size_t index = model.getDOFIndex(name);
    sim.positions()(index) = model.getDOF(name);
  }

  // Full 6 DOFs constraints on the ground
  Leph::RBDL::ConstraintSet constraints;
  constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
  // Z 0
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("left_cleat_1")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 1
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("left_cleat_1")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Y 2
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("left_cleat_1")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));
  // Z 3
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("left_cleat_2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 4
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("left_cleat_2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Z 5
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("left_cleat_3")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  constraints.Bind(model.getRBDLModel());

  Leph::ModelViewer viewer(1200, 900);
  Leph::Plot plot;
  size_t count = 0;
  while (viewer.update())
  {
    std::cout << "********** count=" << count << std::endl;
    sim.update(0.0001, &constraints);
    count++;
    Leph::VectorLabel vect;
    for (size_t i = 0; i < model.sizeDOF(); i++)
    {
      vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
      vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
      vect.append("joint:" + model.getDOFName(i), sim.jointTorques()(i));
      vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
      vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
      vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
      vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
    }
    vect.append("force:1", constraints.force(0));
    vect.append("force:2", constraints.force(3));
    vect.append("force:3", constraints.force(5));
    plot.add(vect);
    model.setDOFVect(sim.positions());
    Leph::ModelDraw(model, viewer);
  }
  plot.plot("index", "force:*").render();
  plot.plot("index", "acc:*").render();
  plot.plot("index", "joint:*").render();
  plot.plot("index", "joint:*").plot("index", "friction:*").plot("index", "control:*").render();
  plot.plot("index", "pos:*").render();
  plot.plot("index", "vel:*").render();
  plot.plot("index", "goal:*").render();
}

/**
 * Full Sigmaban model. Floating base on the
 * left foot and 6 DOFs contact constraints
 * on right foot.
 */
void testHumanoidFullConstraints()
{
  // Full humanoid
  Leph::HumanoidModel model(Leph::SigmabanModel, "left_foot_tip", false);

  // Simulator
  Leph::ForwardSimulation sim(model);

  // Single support static position
  Eigen::Vector3d trunkPos(0.0, -0.05, 0.25);
  Eigen::Vector3d trunkAxis(0.0, 0.0, 0.0);
  Eigen::Vector3d footPos(0.0, -0.1, 0.0);
  Leph::HumanoidFixedModel goalModel(Leph::SigmabanModel);
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
    size_t index = model.getDOFIndex(name);
    sim.positions()(index) = goalModel.get().getDOF(name);
    sim.goals()(index) = goalModel.get().getDOF(name);
    model.setDOF(index, goalModel.get().getDOF(name));
  }

  // Full 6 DOFs constraints on the ground
  Leph::RBDL::ConstraintSet constraints;
  constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
  // Z 0
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("right_cleat_1")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 1
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("right_cleat_1")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Y 2
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("right_cleat_1")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));
  // Z 3
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("right_cleat_2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  // X 4
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("right_cleat_2")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
  // Z 5
  constraints.AddConstraint(model.frameIndexToBodyId(model.getFrameIndex("right_cleat_3")),
                            Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0), Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
  constraints.Bind(model.getRBDLModel());

  Leph::ModelViewer viewer(1200, 900);
  Leph::Plot plot;
  size_t count = 0;
  double t = 0.0;
  while (viewer.update())
  {
    std::cout << "********** count=" << count << std::endl;
    t += 0.0001;
    sim.goals()(model.getDOFIndex("left_shoulder_pitch")) = 0.4 * sin(2.0 * 3.14 * t);
    sim.update(0.0001, &constraints);
    count++;
    Leph::VectorLabel vect;
    for (size_t i = 0; i < model.sizeDOF(); i++)
    {
      vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
      vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
      vect.append("joint:" + model.getDOFName(i), sim.jointTorques()(i));
      vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
      vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
      vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
      vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
    }
    vect.append("force:1", constraints.force(0));
    vect.append("force:2", constraints.force(3));
    vect.append("force:3", constraints.force(5));
    plot.add(vect);
    model.setDOFVect(sim.positions());
    Leph::ModelDraw(model, viewer);
  }
  plot.plot("index", "force:*").render();
  plot.plot("index", "acc:*").render();
  plot.plot("index", "joint:*").render();
  plot.plot("index", "joint:*").plot("index", "friction:*").plot("index", "control:*").render();
  plot.plot("index", "pos:*").render();
  plot.plot("index", "vel:*").render();
  plot.plot("index", "goal:*").render();
}

int main()
{
  testForward();
  testConstraints();
  testFullConstraints();
  testHumanoid();
  testHumanoidFullConstraints();
}
