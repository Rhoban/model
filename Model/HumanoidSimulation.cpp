#include <iostream>
#include <stdexcept>
#include "Model/HumanoidSimulation.hpp"
#include "Utils/Euler.h"

namespace Leph
{
HumanoidSimulation::HumanoidSimulation(RobotType type, const Eigen::MatrixXd& inertiaData,
                                       const std::map<std::string, size_t>& inertiaName,
                                       const Eigen::MatrixXd& geometryData,
                                       const std::map<std::string, size_t>& geometryName)
  : _model(type, "trunk", true, inertiaData, inertiaName, geometryData, geometryName)
  , _simulation(_model)
  , _isInitialized(false)
  , _constraints()
  , _cleats()
  , _isFixedContact(false)
{
  // Model initialization
  putOnGround();
  putFootAt(0.0, 0.0);
  // Foot cleats initialization
  addCleat("left_cleat_1");
  addCleat("left_cleat_2");
  addCleat("left_cleat_3");
  addCleat("left_cleat_4");
  addCleat("right_cleat_1");
  addCleat("right_cleat_2");
  addCleat("right_cleat_3");
  addCleat("right_cleat_4");
}

const HumanoidModel& HumanoidSimulation::model() const
{
  return _model;
}
HumanoidModel& HumanoidSimulation::model()
{
  return _model;
}

void HumanoidSimulation::putOnGround()
{
  // Find lowest foot
  Eigen::Vector3d posLeft = _model.position("trunk", "left_foot_tip");
  Eigen::Vector3d posRight = _model.position("trunk", "right_foot_tip");
  // Forward to putOnGround()
  if (posLeft.z() >= posRight.z())
  {
    putOnGround(HumanoidFixedModel::LeftSupportFoot);
  }
  else
  {
    putOnGround(HumanoidFixedModel::RightSupportFoot);
  }
}
void HumanoidSimulation::putOnGround(HumanoidFixedModel::SupportFoot foot)
{
  std::string footName;
  if (foot == HumanoidFixedModel::LeftSupportFoot)
  {
    footName = "left_foot_tip";
  }
  else
  {
    footName = "right_foot_tip";
  }

  // Set the foot flat
  Eigen::Matrix3d rotOriginToTrunk = _model.orientation("trunk", "origin");
  Eigen::Matrix3d rotOriginToFoot = _model.orientation(footName, "origin");
  Eigen::Matrix3d rotation = rotOriginToTrunk * rotOriginToFoot.transpose();
  // Retrieve YawPitchRoll euler angles from rotation matrix
  //(Manual computing without singular check seems better than
  // Eigen euler angles and with better range)
  Eigen::Vector3d angles;
  // Roll
  angles(0) = atan2(rotation(1, 2), rotation(2, 2));
  // Pitch
  angles(1) = atan2(-rotation(0, 2), sqrt(rotation(0, 0) * rotation(0, 0) + rotation(0, 1) * rotation(0, 1)));
  // Yaw
  angles(2) = atan2(rotation(0, 1), rotation(0, 0));
  // Update base
  setPos("base_roll", angles(0));
  setPos("base_pitch", angles(1));
  setPos("base_yaw", angles(2));

  // Set the foot on ground
  Eigen::VectorXd pos = _model.position(footName, "origin");
  double height = getPos("base_z");
  setPos("base_z", height - pos.z());
}

void HumanoidSimulation::putFootAt(double x, double y)
{
  // Find lowest foot
  Eigen::Vector3d posLeft = _model.position("trunk", "left_foot_tip");
  Eigen::Vector3d posRight = _model.position("trunk", "right_foot_tip");
  // Forward to putFootAt()
  if (posLeft.z() >= posRight.z())
  {
    putFootAt(x, y, HumanoidFixedModel::LeftSupportFoot);
  }
  else
  {
    putFootAt(x, y, HumanoidFixedModel::RightSupportFoot);
  }
}
void HumanoidSimulation::putFootAt(double x, double y, HumanoidFixedModel::SupportFoot foot, bool isContactFixed)
{
  bool isLeftFoot;
  std::string footName;
  if (foot == HumanoidFixedModel::LeftSupportFoot)
  {
    footName = "left_foot_tip";
    isLeftFoot = true;
  }
  else
  {
    footName = "right_foot_tip";
    isLeftFoot = false;
  }

  // Compute translation in origin frame
  Eigen::Vector3d originToFoot(x, y, 0.0);
  Eigen::Vector3d footToTrunk = _model.position("trunk", footName);
  Eigen::Vector3d originToTrunk = originToFoot + footToTrunk;

  // Update base position
  setPos("base_x", originToTrunk.x());
  setPos("base_y", originToTrunk.y());

  // Set the foot fixed
  if (isContactFixed)
  {
    _isFixedContact = true;
    unsigned int count = 0;
    for (auto& it : _cleats)
    {
      it.second.isActive = false;
      it.second.isContact = false;
      if (it.second.isLeftFoot == isLeftFoot && count < 3)
      {
        it.second.isActive = true;
        it.second.isContact = true;
        count++;
      }
    }
    _constraints = buildConstraintSet(false, true, true);
  }
}

const Eigen::VectorXd& HumanoidSimulation::positions() const
{
  return _simulation.positions();
}
Eigen::VectorXd& HumanoidSimulation::positions()
{
  return _simulation.positions();
}
const Eigen::VectorXd& HumanoidSimulation::velocities() const
{
  return _simulation.velocities();
}
Eigen::VectorXd& HumanoidSimulation::velocities()
{
  return _simulation.velocities();
}
const Eigen::VectorXd& HumanoidSimulation::goals() const
{
  return _simulation.goals();
}
Eigen::VectorXd& HumanoidSimulation::goals()
{
  return _simulation.goals();
}
const Eigen::VectorXd& HumanoidSimulation::jointTorques() const
{
  return _simulation.jointTorques();
}
const Eigen::VectorXd& HumanoidSimulation::frictionTorques() const
{
  return _simulation.frictionTorques();
}
const Eigen::VectorXd& HumanoidSimulation::controlTorques() const
{
  return _simulation.controlTorques();
}
const Eigen::VectorXd& HumanoidSimulation::accelerations() const
{
  return _simulation.accelerations();
}

void HumanoidSimulation::setGoal(const std::string& name, double pos)
{
  _simulation.goals()(_model.getDOFIndex(name)) = pos;
}
void HumanoidSimulation::setPos(const std::string& name, double pos)
{
  _simulation.positions()(_model.getDOFIndex(name)) = pos;
  _model.setDOF(name, pos);
}
void HumanoidSimulation::setVel(const std::string& name, double pos)
{
  _simulation.velocities()(_model.getDOFIndex(name)) = pos;
}

double HumanoidSimulation::getGoal(const std::string& name) const
{
  return _simulation.goals()(_model.getDOFIndex(name));
}
double HumanoidSimulation::getPos(const std::string& name) const
{
  return _simulation.positions()(_model.getDOFIndex(name));
}
double HumanoidSimulation::getVel(const std::string& name) const
{
  return _simulation.velocities()(_model.getDOFIndex(name));
}

const JointModel& HumanoidSimulation::jointModel(const std::string& name) const
{
  return _simulation.jointModel(_model.getDOFIndex(name));
}
JointModel& HumanoidSimulation::jointModel(const std::string& name)
{
  return _simulation.jointModel(_model.getDOFIndex(name));
}

void HumanoidSimulation::setJointModelParameters(const Eigen::VectorXd& params)
{
  _simulation.setJointModelParameters(params);
}

double HumanoidSimulation::getCleatForce(const std::string& name) const
{
  if (_cleats.count(name) == 0)
  {
    throw std::logic_error("HumanoidSimulation invalid cleat name: " + name);
  }
  return _cleats.at(name).force;
}

void HumanoidSimulation::update(double dt)
{
  // std::cout << "########## Step ######### " << dt << std::endl; //XXX

  bool isNeedLCPUpdate = false;
  bool isNeedImpulse = false;
  if (!_isFixedContact)
  {
    checkAndUpdateCleatsState(isNeedLCPUpdate, isNeedImpulse);
  }

  // If needed, apply impulses on velocity
  // to prevent collision
  if (isNeedImpulse)
  {
  }

  // If needed, recompute active constraints set
  if ((isNeedLCPUpdate || !_isInitialized) && !_isFixedContact)
  {
    RBDL::ConstraintSet tmpConstraints = buildConstraintSet(true, true /*XXX*/, false);
    // XXX std::cout << "Compute Impulse !" << std::endl;
    _simulation.computeImpulses(tmpConstraints);

    findActiveConstraintsLCP();
    _isInitialized = true;
  }

  // TODO XXX
  // Check active cleat position
  /*
  std::vector<Eigen::Vector3d> activeCleatsPos;
  size_t indexTODO = 0;
  for (const auto& it : _cleats) {
      if (it.second.isActive) {
          Eigen::Vector3d pos = _model.position(it.second.frame, "origin");
          activeCleatsPos.push_back(pos);
          indexTODO++;
      }
  }
  */

  // Simulation update
  _simulation.update(dt, &_constraints);
  // Assign model position state
  _model.setDOFVect(_simulation.positions());

  // TODO XXX
  // Check active cleat position
  /* XXX
  indexTODO = 0;
  for (const auto& it : _cleats) {
      if (it.second.isActive) {
          Eigen::Vector3d pos = _model.position(it.second.frame, "origin");
          Eigen::Vector3d diff = pos-activeCleatsPos[indexTODO];
          std::cout << it.second.frame << " SLIP===" << diff.lpNorm<Eigen::Infinity>() << std::endl;
          if (diff.lpNorm<Eigen::Infinity>() > 1e-8) {
              //exit(0);
          }
          indexTODO++;
      }
  }
  */

  // Retrieve contact active force
  for (auto& it : _cleats)
  {
    if (it.second.isActive)
    {
      it.second.force = _constraints.force(it.second.index);
    }
    else
    {
      it.second.force = 0.0;
    }
  }
}

void HumanoidSimulation::printCleatsStatus(bool verbose)
{
  if (verbose)
  {
    for (const auto& it : _cleats)
    {
      std::cout << "[" << it.second.frame << ":" << (it.second.isLeftFoot ? "left" : "right")
                << "] pos=" << _model.position(it.second.frame, "origin").z() << " "
                << (it.second.isContact ? "isContact" : "") << " "
                << (it.second.isActive ? "isActive index=" + std::to_string(it.second.index) +
                                             " force=" + std::to_string(it.second.force) :
                                         "")
                << std::endl;
    }
    std::cout << "Constraint set size=" << _constraints.force.size() << std::endl;
  }
  else
  {
    for (const auto& it : _cleats)
    {
      std::cout << " " << _model.position(it.second.frame, "origin").z() << " "
                << _model.pointVelocity(it.second.frame, "origin", velocities())(5) << " " << it.second.force << " "
                << (it.second.isContact ? "1" : "0") << " " << (it.second.isActive ? "1" : "0");
    }
    std::cout << std::endl;
  }
}
void HumanoidSimulation::printCleatsStatus(Plot& plot)
{
  VectorLabel vect;
  for (const auto& it : _cleats)
  {
    vect.append(it.second.frame + ":pos", _model.position(it.second.frame, "origin").z());
    vect.append(it.second.frame + ":vel", _model.pointVelocity(it.second.frame, "origin", velocities())(5));
    vect.append(it.second.frame + ":acc",
                _model.pointAcceleration(it.second.frame, "origin", velocities(), accelerations())(5));
    vect.append(it.second.frame + ":force", it.second.force);
    vect.append(it.second.frame + ":isContact", (it.second.isContact ? 1.0 : 0.0));
    vect.append(it.second.frame + ":isActive", (it.second.isActive ? 1.0 : 0.0));
  }
  plot.add(vect);
}

void HumanoidSimulation::addCleat(const std::string& frame)
{
  size_t bodyId = _model.frameIndexToBodyId(_model.getFrameIndex(frame));
  bool isLeftFoot = (frame.find("left_cleat") != std::string::npos);
  bool isTopSide = (frame.find("cleat_1") != std::string::npos) || (frame.find("cleat_2") != std::string::npos);
  bool isLeftSide = (frame.find("cleat_1") != std::string::npos) || (frame.find("cleat_4") != std::string::npos);
  _cleats[frame] = { frame, bodyId, isLeftFoot, isTopSide, isLeftSide, false, false, (size_t)-1, 0.0, 0.0 };
}

RBDL::ConstraintSet HumanoidSimulation::buildConstraintSet(bool withContact, bool withLateral, bool assignCleats,
                                                           Eigen::VectorXi* isBilateralConstraint)
{
  // Create and init the set
  RBDL::ConstraintSet set;
  set.SetSolver(RBDLMath::LinearSolverFullPivHouseholderQR);
  if (isBilateralConstraint != nullptr)
  {
    *isBilateralConstraint = Eigen::VectorXi();
  }

  // State of selected lateral constraints.
  // Use to not select two constraints on the
  // same foot on the same action line.
  // Possible states are the Cartesian product
  // of:
  // left[0] or right[1] foot
  // left[0] or right[i] side
  bool selectedLateralStateX[2][2] = { false };
  // left[0] or right[1] foot
  // top[0] or bottom[i] side
  bool selectedLateralStateY[2][2] = { false };
  // Count selected vertical and lateral
  // constraints for each foot
  unsigned int countSelectedVertical[2] = { 0 };
  unsigned int countSelectedLateral[2] = { 0 };

  // Loop over all cleats
  size_t indexInSet = 0;
  for (auto& it : _cleats)
  {
    // Select active or contacting cleats
    if (
        // countSelectedVertical[it.second.isLeftFoot] < 3 && //TODO XXX
        (it.second.isActive || (withContact && it.second.isContact)))
    {
      // Create Z constraint
      // for all selected cleat
      // XXX std::cout << "Add Z " << it.second.frame << std::endl;
      set.AddConstraint(it.second.bodyId, RBDLMath::Vector3d(0.0, 0.0, 0.0), RBDLMath::Vector3d(0.0, 0.0, 1.0));
      countSelectedVertical[it.second.isLeftFoot]++;
      if (isBilateralConstraint != nullptr)
      {
        isBilateralConstraint->conservativeResize(indexInSet + 1);
        (*isBilateralConstraint)(indexInSet) = 0;
      }
      // Assign constraint index in set
      if (assignCleats)
      {
        it.second.index = indexInSet;
      }
      indexInSet++;
      if (withLateral)
      {
        // Create X constraints only for the first
        // two active constraints if existing
        if (((countSelectedVertical[it.second.isLeftFoot] == 1 && countSelectedLateral[it.second.isLeftFoot] < 2) ||
             (countSelectedVertical[it.second.isLeftFoot] == 2 && countSelectedLateral[it.second.isLeftFoot] < 3) ||
             (countSelectedVertical[it.second.isLeftFoot] == 3 && countSelectedLateral[it.second.isLeftFoot] < 3)) &&
            !selectedLateralStateX[it.second.isLeftFoot][it.second.isLeftSide])
        {
          // XXX std::cout << "Add X " << it.second.frame << std::endl;
          set.AddConstraint(it.second.bodyId, RBDLMath::Vector3d(0.0, 0.0, 0.0), RBDLMath::Vector3d(1.0, 0.0, 0.0));
          countSelectedLateral[it.second.isLeftFoot]++;
          selectedLateralStateX[it.second.isLeftFoot][it.second.isLeftSide] = true;
          if (isBilateralConstraint != nullptr)
          {
            isBilateralConstraint->conservativeResize(indexInSet + 1);
            (*isBilateralConstraint)(indexInSet) = 1;
          }
          indexInSet++;
        }
        // Create Y constraints only for the first
        // active constraint if existing
        if (((countSelectedVertical[it.second.isLeftFoot] == 1 && countSelectedLateral[it.second.isLeftFoot] < 2) ||
             (countSelectedVertical[it.second.isLeftFoot] == 2 && countSelectedLateral[it.second.isLeftFoot] < 3) ||
             (countSelectedVertical[it.second.isLeftFoot] == 3 && countSelectedLateral[it.second.isLeftFoot] < 3)) &&
            !selectedLateralStateY[it.second.isLeftFoot][it.second.isTopSide])
        {
          // XXX std::cout << "Add Y " << it.second.frame << std::endl;
          set.AddConstraint(it.second.bodyId, RBDLMath::Vector3d(0.0, 0.0, 0.0), RBDLMath::Vector3d(0.0, 1.0, 0.0));
          countSelectedLateral[it.second.isLeftFoot]++;
          selectedLateralStateY[it.second.isLeftFoot][it.second.isTopSide] = true;
          if (isBilateralConstraint != nullptr)
          {
            isBilateralConstraint->conservativeResize(indexInSet + 1);
            (*isBilateralConstraint)(indexInSet) = 1;
          }
          indexInSet++;
        }
      }
      // 1 selected cleat : 3 foot dof -> 3 constraints
      //(1Z, 1X, 1Y)
      // 2 selected cleat : 1 foot dof -> 5 constraints
      //(2Z, 2X, 1Y) or (2Z, 1X, 2Y)
      // 3 selected cleat : 0 foot dof -> 6 constraints
      //(3Z, 2X, 1Y) or (3Z, 1X, 2Y)
      // To prevent two constraints to work
      // again each other, the action line
      // of two constraints must not be collinear.
      // For example, the X constraint must not
      // be activated on both right top and right
      // bottom cleats on the same foot because the
      // force vectors would be on the same line.
    }
    // TODO assign -1 in indexInSet for 4th contacting cleat
    // and update in loop contact to active not to active
    // the 4th cleat
  }
  // Bind and initialize the set
  // with the RBDL model
  set.Bind(_model.getRBDLModel());

  return set;
}

void HumanoidSimulation::checkAndUpdateCleatsState(bool& isNeedLCPUpdate, bool& isNeedImpulse)
{
  // Iterate over all cleats
  isNeedLCPUpdate = false;
  isNeedImpulse = false;
  for (auto& it : _cleats)
  {
    // Compute cleat vertical Z position
    Eigen::Vector3d pos = _model.position(it.second.frame, "origin");
    // Check non contacting cleats for collision
    if (!it.second.isContact && pos.z() <= 0.0)
    {
      // A collision is detected
      it.second.isContact = true;
      it.second.isActive = false;
      it.second.force = 0.0;
      isNeedLCPUpdate = true;
      isNeedImpulse = true;
      // XXX std::cout << "Collision for " << it.second.frame << " Z=" << pos.z() << std::endl; //XXX
      // Check cleats for contact lost
    }
    else if (it.second.isContact && !it.second.isActive && pos.z() > 0.0)
    {
      // The contact is released
      it.second.isContact = false;
      it.second.isActive = false;
      it.second.force = 0.0;
      // No need for ConstraintSet update
      // XXX std::cout << "Detaching for " << it.second.frame << " Z=" << pos.z() << std::endl; //TODO
      // Check contacting cleats sinking
    }
    else if (it.second.isContact && !it.second.isActive && pos.z() < it.second.height - 1e-7)
    {
      // Recompute active constraints to try to
      // prevent contact cleat to sink
      isNeedLCPUpdate = true;
      isNeedImpulse = true;
      // XXX std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Sinking for " << it.second.frame << " Z=" << pos.z() << "
      // lastZ=" << it.second.height << " Delta=" << it.second.height - pos.z() << std::endl; //TODO
      // Check for negative force
    }
    else if (it.second.isActive && it.second.force < 0.0)
    {
      isNeedLCPUpdate = true;
      // XXX std::cout << "Negative force for " << it.second.frame << std::endl; //XXX
    }
    // Update cleat height for sinking detection
    it.second.height = pos.z();
  }
}

void HumanoidSimulation::findActiveConstraintsLCP()
{
  // ConstraintSet allocation
  Eigen::VectorXi isBilateralConstraint;
  RBDL::ConstraintSet tmpConstraints = buildConstraintSet(true, true /*XXX*/, true, &isBilateralConstraint);
  _simulation.computeContactLCP(tmpConstraints, isBilateralConstraint);
  // XXX std::cout << "HumanoidSimulation LCP lambda=" << tmpConstraints.force.transpose() << std::endl;

  for (auto& it : _cleats)
  {
    if (it.second.isContact)
    {
      it.second.isActive = false;
      if (tmpConstraints.force(it.second.index) > 0.0)
      {
        it.second.isActive = true;
        /* XXX
        std::cout << "Enabling " << it.second.frame
            << " with force=" << tmpConstraints.force(it.second.index) << std::endl;
        */
      }
    }
  }

  _constraints = buildConstraintSet(false, true /*XXX*/, true);

  /*
  std::cout << "Re Compute final Impulse !" << std::endl;
  _simulation.computeImpulses(_constraints);
  */
}

}  // namespace Leph
