#include "Concepts/HumanoidSensorsModelConcept.hpp"

namespace Leph
{
HumanoidSensorsModelConcept::HumanoidSensorsModelConcept(RobotType type) : _model(type)
{
}

std::string HumanoidSensorsModelConcept::name() const
{
  return "HumanoidSensorsModelConcept";
}
size_t HumanoidSensorsModelConcept::inputSize() const
{
  return 30;
}
size_t HumanoidSensorsModelConcept::outputSize() const
{
  return 6;
}

size_t HumanoidSensorsModelConcept::parameterSize() const
{
  return 0;
}
Leph::MetaParameter HumanoidSensorsModelConcept::defaultParameter(size_t index) const
{
  (void)index;
  return MetaParameter();
}

bool HumanoidSensorsModelConcept::doCompute(double time)
{
  // Check if all input values are available
  for (size_t i = 0; i < inputSize(); i++)
  {
    if (!Concept::getInput(i)->isTimeValid(time))
    {
      return false;
    }
  }
  // Check if all output series are
  // available for writing
  for (size_t i = 0; i < outputSize(); i++)
  {
    if (Concept::getOutput(i)->size() > 0 && Concept::getOutput(i)->timeMax() >= time)
    {
      return false;
    }
  }

  // Update degrees of freedom in the model
  _model.get().setDOF("left_ankle_roll", Concept::getInput(0)->get(time));
  _model.get().setDOF("left_ankle_pitch", Concept::getInput(1)->get(time));
  _model.get().setDOF("left_knee", Concept::getInput(2)->get(time));
  _model.get().setDOF("left_hip_pitch", Concept::getInput(3)->get(time));
  _model.get().setDOF("left_hip_roll", Concept::getInput(4)->get(time));
  _model.get().setDOF("left_hip_yaw", Concept::getInput(5)->get(time));
  _model.get().setDOF("right_ankle_roll", Concept::getInput(6)->get(time));
  _model.get().setDOF("right_ankle_pitch", Concept::getInput(7)->get(time));
  _model.get().setDOF("right_knee", Concept::getInput(8)->get(time));
  _model.get().setDOF("right_hip_pitch", Concept::getInput(9)->get(time));
  _model.get().setDOF("right_hip_roll", Concept::getInput(10)->get(time));
  _model.get().setDOF("right_hip_yaw", Concept::getInput(11)->get(time));
  _model.get().setDOF("left_shoulder_pitch", Concept::getInput(12)->get(time));
  _model.get().setDOF("left_shoulder_roll", Concept::getInput(13)->get(time));
  _model.get().setDOF("left_elbow", Concept::getInput(14)->get(time));
  _model.get().setDOF("right_shoulder_pitch", Concept::getInput(15)->get(time));
  _model.get().setDOF("right_shoulder_roll", Concept::getInput(16)->get(time));
  _model.get().setDOF("right_elbow", Concept::getInput(17)->get(time));
  _model.get().setDOF("head_yaw", Concept::getInput(18)->get(time));
  _model.get().setDOF("head_pitch", Concept::getInput(19)->get(time));
  // Set feet pressure
  _model.setPressure(Concept::getInput(23)->get(time), Concept::getInput(24)->get(time),
                     Concept::getInput(25)->get(time), Concept::getInput(26)->get(time),
                     Concept::getInput(27)->get(time), Concept::getInput(28)->get(time),
                     Concept::getInput(29)->get(time));
  // Update support foot and compute odometry
  _model.updateBase();
  // Update trunk orientation using IMU
  // and gyro integration
  _model.setOrientation(
      Eigen::AngleAxisd(Concept::getInput(21)->get(time), Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(Concept::getInput(20)->get(time), Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(Concept::getInput(22)->getAngular(time), Eigen::Vector3d::UnitZ()).toRotationMatrix());

  // Write output is_support_foot_left
  double is_support_foot_left = (_model.getSupportFoot() == Leph::HumanoidFixedModel::LeftSupportFoot) ? 1.0 : 0.0;
  // Special behaviour, support foot series is appended only
  // when support foot is swap
  if (Concept::getOutput(0)->size() == 0 || Concept::getOutput(0)->lastValue() != is_support_foot_left)
  {
    // Compute support length
    if (Concept::getOutput(0)->size() > 0)
    {
      double supportLength = time - Concept::getOutput(0)->lastTime();
      Concept::getOutput(5)->append(time, supportLength);
    }
    // Append support foot
    Concept::getOutput(0)->append(time, is_support_foot_left);
  }

  // Write output head pose
  Eigen::Vector3d headPos = _model.get().position("camera", "origin");
  Concept::getOutput(1)->append(time, headPos.x());
  Concept::getOutput(2)->append(time, headPos.y());
  Concept::getOutput(3)->append(time, headPos.z());
  Concept::getOutput(4)->append(time, _model.get().orientationYaw("camera", "origin"));

  return true;
}

}  // namespace Leph
