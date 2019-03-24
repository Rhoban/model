#include <stdexcept>
#include "Model/HumanoidFixedPressureModel.hpp"

namespace Leph
{
HumanoidFixedPressureModel::HumanoidFixedPressureModel(RobotType type, const Eigen::MatrixXd& inertiaData,
                                                       const std::map<std::string, size_t>& inertiaName,
                                                       const Eigen::MatrixXd& geometryData,
                                                       const std::map<std::string, size_t>& geometryName)
  : HumanoidFixedModel(type, inertiaData, inertiaName, geometryData, geometryName)
  , _weight(0.0)
  , _leftRatio(1.0)
  , _rightRatio(0.0)
  , _leftCOP(Eigen::Vector3d::Zero())
  , _rightCOP(Eigen::Vector3d::Zero())
{
}

void HumanoidFixedPressureModel::setPressure(const Leph::VectorLabel& vect)
{
  for (size_t i = 0; i < vect.size(); i++)
  {
    const std::string& label = vect.getLabel(i);
    if (label == "weight")
      _weight = vect(i);
    if (label == "left_weight")
      _leftRatio = vect(i);
    if (label == "right_weight")
      _rightRatio = vect(i);
    if (label == "left_x")
      _leftCOP.x() = vect(i);
    if (label == "right_x")
      _rightCOP.x() = vect(i);
    if (label == "left_y")
      _leftCOP.y() = vect(i);
    if (label == "right_y")
      _rightCOP.y() = vect(i);
  }

  // Checking inputs
  if (_weight < 0.0 || _leftRatio < 0.0 || _rightRatio < 0.0)
  {
    throw std::logic_error("HumanoidFixedPressureModel invalid pressure inputs");
  }
  checkPressure();
}
void HumanoidFixedPressureModel::setPressure(double weight, double leftRatio, double rightRatio, double leftX,
                                             double leftY, double rightX, double rightY)
{
  // Checking inputs
  if (weight < 0.0 || leftRatio < 0.0 || rightRatio < 0.0)
  {
    throw std::logic_error("HumanoidFixedPressureModel invalid pressure inputs");
  }

  // Assigning pressure state
  _weight = weight;
  _leftRatio = leftRatio;
  _rightRatio = rightRatio;
  _leftCOP.x() = leftX;
  _leftCOP.y() = leftY;
  _leftCOP.z() = 0.0;
  _rightCOP.x() = rightX;
  _rightCOP.y() = rightY;
  _rightCOP.z() = 0.0;

  // Check inputs ratio
  checkPressure();
}

void HumanoidFixedPressureModel::updateBase()
{
  // Set support foot to foot which pressure is maximum
  if (_leftRatio > _rightRatio)
  {
    HumanoidFixedModel::setSupportFoot(LeftSupportFoot);
  }
  else
  {
    HumanoidFixedModel::setSupportFoot(RightSupportFoot);
  }
}

double HumanoidFixedPressureModel::pressureWeight() const
{
  return _weight;
}
double HumanoidFixedPressureModel::pressureLeftWeight() const
{
  return _weight * _leftRatio;
}
double HumanoidFixedPressureModel::pressureRightWeight() const
{
  return _weight * _rightRatio;
}
double HumanoidFixedPressureModel::pressureLeftRatio() const
{
  return _leftRatio;
}
double HumanoidFixedPressureModel::pressureRightRatio() const
{
  return _rightRatio;
}

Eigen::Vector3d HumanoidFixedPressureModel::centerOfPressureLeft(const std::string& frame)
{
  // Retrieve foot center
  double sizeX;
  double sizeY;
  double sizeZ;
  Eigen::Vector3d center;
  HumanoidFixedModel::get().boundingBox(HumanoidFixedModel::get().getFrameIndex("left_foot_tip"), sizeX, sizeY, sizeZ,
                                        center);

  // Compute center of pressure
  return HumanoidFixedModel::get().position("left_foot_tip", frame, _leftCOP + center);
}
Eigen::Vector3d HumanoidFixedPressureModel::centerOfPressureRight(const std::string& frame)
{
  // Retrieve foot center
  double sizeX;
  double sizeY;
  double sizeZ;
  Eigen::Vector3d center;
  HumanoidFixedModel::get().boundingBox(HumanoidFixedModel::get().getFrameIndex("right_foot_tip"), sizeX, sizeY, sizeZ,
                                        center);

  // Compute center of pressure
  return HumanoidFixedModel::get().position("right_foot_tip", frame, _rightCOP + center);
}
Eigen::Vector3d HumanoidFixedPressureModel::centerOfPressure(const std::string& frame)
{
  // Compute barycenter of each foot center of pressure
  Eigen::Vector3d left = centerOfPressureLeft(frame);
  Eigen::Vector3d right = centerOfPressureRight(frame);

  return _leftRatio * left + _rightRatio * right;
}

void HumanoidFixedPressureModel::checkPressure()
{
  if (_leftRatio > 1.0)
  {
    std::cerr << "Warning: HumanoidFixedPressureModel invalid left ratio: " << _leftRatio << std::endl;
    _leftRatio = 1.0;
    _rightRatio = 0.0;
  }
  if (_rightRatio > 1.0)
  {
    std::cerr << "Warning: HumanoidFixedPressureModel invalid right ratio: " << _leftRatio << std::endl;
    _rightRatio = 1.0;
    _leftRatio = 0.0;
  }
}

}  // namespace Leph
