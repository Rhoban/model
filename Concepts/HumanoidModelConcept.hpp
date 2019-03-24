#ifndef LEPH_HUMANOIDMODELCONCEPT_HPP
#define LEPH_HUMANOIDMODELCONCEPT_HPP

#include "TimeSeries/Concept.hpp"
#include "Model/HumanoidFixedModel.hpp"

namespace Leph
{
/**
 * HumanoidModelConcept
 *
 * Implement the robot Humanoid
 * Fixed Model (without sensors) as a Concept.
 * Inputs:
 * 0:  left_ankle_roll
 * 1:  left_ankle_pitch
 * 2:  left_knee
 * 3:  left_hip_pitch
 * 4:  left_hip_roll
 * 5:  left_hip_yaw
 * 6:  right_ankle_roll
 * 7:  right_ankle_pitch
 * 8:  right_knee
 * 9:  right_hip_pitch
 * 10: right_hip_roll
 * 11: right_hip_yaw
 * 12: left_shoulder_pitch
 * 13: left_shoulder_roll
 * 14: left_elbow
 * 15: right_shoulder_pitch
 * 16: right_shoulder_roll
 * 17: right_elbow
 * 18: head_yaw
 * 19: head_pitch
 * Outputs:
 * 0:  is_support_foot_left
 * 1:  head_x
 * 2:  head_y
 * 3:  head_z
 * 4:  head_theta
 * 5:  support_length
 */
class HumanoidModelConcept : public Concept
{
public:
  /**
   * Initialization with robot type
   */
  HumanoidModelConcept(RobotType type);

  /**
   * Inherit Concept
   */
  virtual std::string name() const override;
  virtual size_t inputSize() const override;
  virtual size_t outputSize() const override;

  /**
   * Inherit Optimize
   */
  virtual size_t parameterSize() const override;
  virtual Leph::MetaParameter defaultParameter(size_t index) const override;

protected:
  /**
   * Inherit Concept
   */
  virtual bool doCompute(double time) override;

private:
  /**
   * Internal Humanoid model
   */
  HumanoidFixedModel _model;
};

}  // namespace Leph

#endif
