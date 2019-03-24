#ifndef LEPH_QUINTICWALK_HPP
#define LEPH_QUINTICWALK_HPP

#include <Eigen/Dense>
#include "Types/VectorLabel.hpp"
#include "QuinticWalk/Footstep.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"

namespace Leph
{
/**
 * QuinticWalk
 *
 * Holonomic and open loop walk
 * generator based on footstep control
 * and quintic splines in cartesian space.
 * Expressed all target state in cartesian
 * space with respect to current cupport foot
 */
class QuinticWalk
{
public:
  /**
   * Initialization
   */
  QuinticWalk();

  /**
   * Return current walk phase
   * between 0 and 1
   */
  double getPhase() const;

  /**
   * Return current time between
   * 0 and half period for
   * trajectories evaluation
   */
  double getTrajsTime() const;

  /**
   * Get current used
   * parameters
   */
  const VectorLabel& getParameters() const;

  /**
   * Get current walk footstep orders
   */
  const Eigen::Vector3d& getOrders() const;

  /**
   * Return true is the walk
   * oscillations are enabled
   */
  bool isEnabled() const;

  /**
   * Assign given parameters vector
   */
  void setParameters(const VectorLabel& params);

  /**
   * Rebuilt the trajectories and
   * reset saved state as disable.
   * Used to directly apply
   * newly parameters.
   */
  void forceRebuildTrajectories();

  /**
   * Set used walk footstep orders,
   * enable or disable the walk oscillations
   * and optionnaly set the starting
   * supporting foot.
   */
  void setOrders(const Eigen::Vector3d& orders, bool isEnabled, bool beginWithLeftSupport = true);

  /**
   * Return the trajectories for
   * current half cycle
   */
  const Trajectories& getTrajectories() const;

  /**
   * Update the internal walk state
   * (pĥase, trajectories) from given
   * elapsed time since last update() call
   */
  void update(double dt);

  /**
   * Compute current cartesian
   * target from trajectories and assign
   * it to given model through inverse
   * kinematics.
   * Return false is the target is
   * unreachable.
   */
  bool assignModel(HumanoidFixedModel& model);

private:
  /**
   * Current footstep support
   * and flying last and next pose
   */
  Footstep _footstep;

  /**
   * Movement phase between 0 and 1
   */
  double _phase;

  /**
   * Currently used parameters
   */
  VectorLabel _params;

  /**
   * Currently used footstep
   * orders flush at next suppot
   * foot swap
   */
  Eigen::Vector3d _orders;

  /**
   * Enable or disable
   * the oscillations and
   * value at last half cycle.
   */
  bool _isEnabled;
  bool _wasEnabled;

  /**
   * True if the current used
   * trajectories has oscillations
   */
  bool _isTrajsOscillating;

  /**
   * Trunk pose and orientation
   * position, velocity and acceleration
   * at half cycle start
   */
  Eigen::Vector3d _trunkPosAtLast;
  Eigen::Vector3d _trunkVelAtLast;
  Eigen::Vector3d _trunkAccAtLast;
  Eigen::Vector3d _trunkAxisPosAtLast;
  Eigen::Vector3d _trunkAxisVelAtLast;
  Eigen::Vector3d _trunkAxisAccAtLast;

  /**
   * Generated half walk
   * cycle trajectory
   */
  Trajectories _trajs;

  /**
   * Reset and rebuild the
   * spline trajectories for
   * current half cycle
   */
  void buildTrajectories();

  /**
   * Reset the trunk position and
   * orientation state vectors at last
   * half cycle as stopped pose
   */
  void resetTrunkLastState();
};

}  // namespace Leph

#endif
