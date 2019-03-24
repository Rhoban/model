#include "Model/HumanoidFloatingModel.hpp"
#include "Utils/Euler.h"

namespace Leph
{
HumanoidFloatingModel::HumanoidFloatingModel(RobotType type)
  : HumanoidModel(type, "ROOT"), _supportFoot(LeftSupportFoot), _statePosX(0.0), _statePosY(0.0), _stateRotYaw(0.0)
{
  // Center initial position of the model
  _statePosY = Model::position("left_foot_tip", "origin").y();
}

HumanoidFloatingModel::HumanoidFloatingModel(const std::string& urdfFile, RobotType type)
  : HumanoidModel(urdfFile, type, "ROOT")
  , _supportFoot(LeftSupportFoot)
  , _statePosX(0.0)
  , _statePosY(0.0)
  , _stateRotYaw(0.0)
{
  // Center initial position of the model
  _statePosY = Model::position("left_foot_tip", "origin").y();
}

HumanoidFloatingModel::SupportFoot HumanoidFloatingModel::getSupportFoot() const
{
  return _supportFoot;
}

std::string HumanoidFloatingModel::supportFootName() const
{
  if (_supportFoot == LeftSupportFoot)
  {
    return "left_foot_tip";
  }
  else if (_supportFoot == RightSupportFoot)
  {
    return "right_foot_tip";
  }
  else
  {
    return "";
  }
}
std::string HumanoidFloatingModel::movingFootName() const
{
  if (_supportFoot == LeftSupportFoot)
  {
    return "right_foot_tip";
  }
  else if (_supportFoot == RightSupportFoot)
  {
    return "left_foot_tip";
  }
  else
  {
    return "";
  }
}

void HumanoidFloatingModel::putOnGround()
{
  putSupportFootFlat();
  putSupportFootOrigin();
  findSupportFoot();
}

void HumanoidFloatingModel::setStatePosX(double val)
{
  _statePosX = val;
}
void HumanoidFloatingModel::setStatePosY(double val)
{
  _statePosY = val;
}
void HumanoidFloatingModel::setStateRotYaw(double val)
{
  _stateRotYaw = val;
}

void HumanoidFloatingModel::findSupportFoot()
{
  // Feet position in trunk frame
  Eigen::Vector3d posLeftFoot = Model::position("left_foot_tip", "origin");
  Eigen::Vector3d posRightFoot = Model::position("right_foot_tip", "origin");

  // Select the lowest foot in origin frame
  bool swapSupportFoot = false;
  if (posLeftFoot.z() < posRightFoot.z())
  {
    if (_supportFoot == RightSupportFoot)
    {
      swapSupportFoot = true;
    }
    _supportFoot = LeftSupportFoot;
  }
  else
  {
    if (_supportFoot == LeftSupportFoot)
    {
      swapSupportFoot = true;
    }
    _supportFoot = RightSupportFoot;
  }

  // If support foot is changed, the target position
  // of new support foot (which must be fixed in world frame)
  // is update with last position and orientation of
  // old moving foot (same foot)
  if (swapSupportFoot)
  {
    // Moving foot position in world frame
    Eigen::Vector3d posFootWorld = Model::position(supportFootName(), "origin");
    // Updating state
    _statePosX = posFootWorld.x();
    _statePosY = posFootWorld.y();
    _stateRotYaw = Model::orientationYaw(supportFootName(), "origin");
  }
}

void HumanoidFloatingModel::putSupportFootFlat()
{
  // Transformation matrix expressing support foot
  // unit vector into trunk frame
  Eigen::Matrix3d rotation = Model::orientation(supportFootName(), "trunk");
  // Convertion of this rotation matrix into yaw, pitch, roll
  // intrinsic euler angles
  Eigen::Vector3d angles = MatrixToEuler(rotation, EulerRollPitchYaw);
  // Updating floating joint roll, pitch, yaw
  // and apply yaw state rotation
  Model::setDOF("base_roll", angles(0));
  Model::setDOF("base_pitch", angles(1));
  Model::setDOF("base_yaw", angles(2) + _stateRotYaw);
}

void HumanoidFloatingModel::putSupportFootOrigin()
{
  // Support foot position in trunk frame
  Eigen::Vector3d posFoot = Model::position(supportFootName(), "trunk");
  // Convertion of TF vector (T := trunk base origin, F := support foot tip)
  // from trunk frame to world frame
  Eigen::Matrix3d rotation = Model::orientation("origin", "trunk");
  posFoot = rotation * posFoot;
  // Apply translations
  Model::setDOF("base_x", -posFoot.x() + _statePosX);
  Model::setDOF("base_y", -posFoot.y() + _statePosY);
  Model::setDOF("base_z", -posFoot.z());
}

}  // namespace Leph
