#ifndef LEPH_STATICWALK_HPP
#define LEPH_STATICWALK_HPP

#include "Types/VectorLabel.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Model/InverseKinematics.hpp"

namespace Leph
{
/**
 * StaticWalk
 */
class StaticWalk
{
public:
  /**
   * Initialization with robot type
   */
  StaticWalk(RobotType type);

  /**
   * Build and return initial parameters values
   */
  VectorLabel buildParams() const;

  /**
   * Get and set the walk phase
   * (between 0 and 1)
   */
  double getPhase() const;
  void setPhase(double phase);

  /**
   * Return motor output for
   * walk starting pose using
   * given parameters
   */
  VectorLabel initPose(const VectorLabel& params);

  /**
   * Update the walk phase, compute and
   * return the new motors outputs using
   * given time step and parameters
   */
  VectorLabel exec(double dt, const VectorLabel& params);

private:
  /**
   * Movement cyclic phase
   */
  double _phase;

  /**
   * Humanoid model instance
   */
  HumanoidFloatingModel _model;

  /**
   * Inverse Kinematics instance
   */
  InverseKinematics _inverseModel;

  /**
   * Initial position of COM height
   * and foot Y offset in world frame
   */
  double _initCOMZOffset;
  double _initFootYOffset;

  /**
   * Check Inverse Kinematics error
   * and throw std::runtime_error if
   * convergence is failed and if throwError
   * is true.
   * Return if convergence is successful
   */
  bool checkIKErrors(const InverseKinematics& inv, bool throwError) const;

  /**
   * Initialize given Inverse Kinematics
   * instance
   * Trunk DOF is enabled if the flag is true
   */
  void initIK(InverseKinematics& inv, bool enableTrunkOrientation);
};

}  // namespace Leph

#endif
