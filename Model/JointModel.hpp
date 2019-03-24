#ifndef LEPH_JOINTMODEL_HPP
#define LEPH_JOINTMODEL_HPP

#include <string>
#include <Eigen/Dense>
#include <queue>

namespace Leph
{
/**
 * JointModel
 *
 * Mechanical and control model of
 * a joint for its use in Forward Simulation.
 * Multiple joint types are implemented.
 */
class JointModel
{
public:
  /**
   * Initialization with
   * jont type and name
   */
  JointModel(const std::string& name = "");

  /**
   * Return joint textual name
   */
  const std::string& getName() const;

  /**
   * Get and set internal
   * model parameters
   */
  const Eigen::VectorXd getParameters() const;
  void setParameters(const Eigen::VectorXd& params);

  /**
   * Return internal joint inertia
   */
  double getInertia() const;

  /**
   * Set or return the maximum
   * available voltage
   */
  void setMaxVoltage(double volt);
  double getMaxVoltage() const;

  /**
   * Compute the torque applied on
   * the joint by the mechanical friction
   * given current joint position and velocity.
   */
  double frictionTorque(double vel) const;

  /**
   * Compute the torque applied on
   * the joint by the control algorithm
   * given current joint target position,
   * current position and velocity.
   */
  double controlTorque(double pos, double vel) const;

  /**
   * Update current joint friction and
   * control state from given simulation
   * step, target goal position and current state
   */
  void updateState(double dt, double goal, double pos, double vel);

  /**
   * Return current delayed goal
   */
  double getDelayedGoal() const;

  /**
   * Return current backlash hidden state
   */
  bool getBacklashStateEnabled() const;
  double getBacklashStatePos() const;
  double getBacklashStateVel() const;

  /**
   * Reset the backlash state
   * enabled, position and velocity
   * as well as goal lag
   */
  void resetHiddenState();

  /**
   * Optionnaly update given current joint
   * position and velocity to ensure constraints
   */
  void boundState(double& pos, double& vel);

  /**
   * Compute and return electric tension
   * expected to be produced by the motor
   * while following given velocity,
   * acceleration and external torque.
   */
  double computeElectricTension(double velGoal, double accGoal, double torqueGoal) const;

  /**
   * Compute and return the target position offset
   * to be added to goal position to follow a given
   * trajectories.
   * Velocity and acceleration
   * to follow are given.
   * External torque to follow (computed through
   * inverse dynamics) is given.
   */
  double computeFeedForward(double velGoal, double accGoal, double torqueGoal) const;

  /**
   * Print all parameter values on
   * standard output
   */
  void printParameters() const;

private:
  /**
   * Joint textual name
   */
  std::string _name;

  /**
   * Modelisation optional
   * features.
   * Backlash model.
   * Read position encoder discretization.
   * Stribeck friction model.
   * Enable optimization of parameters
   * electric voltage, electric resistance,
   * static friction regularization coefficient.
   */
  bool _featureBacklash;
  bool _featureFrictionStribeck;
  bool _featureReadDiscretization;
  bool _featureOptimizationVoltage;
  bool _featureOptimizationResistance;
  bool _featureOptimizationRegularization;
  bool _featureOptimizationControlGain;

  /**
   * Current integrated time in seconds
   * and target goal history (for lag
   * implementation)
   */
  double _goalTime;
  std::queue<std::pair<double, double>> _goalHistory;

  /**
   * Is backlash state initialized.
   * Hidden states are initialized in first
   * updateState() call.
   */
  bool _isInitialized;

  /**
   * Backlash hidden relative
   * enable and position state
   */
  bool _stateBacklashIsEnabled;
  double _stateBacklashPosition;
  double _stateBacklashVelocity;

  /**
   * Firmware related coefficent
   * from angular position error
   * to PWM ratio
   */
  double _coefAnglePosToPWM;

  /**
   * Firmware related coefficient
   * bounding the PWM control ratio
   * between -100%*_coefPWMBound to
   * 100%*_coefPWMBound.
   */
  double _coefPWMBound;

  /**
   * Model parameters.
   * All positive range value are valid.
   */
  // Friction static regularization coeficient
  double _paramFrictionRegularization;
  // Friction Stribeck transtition velocity
  double _paramFrictionVelLimit;
  // Friction and inertia internal parameters
  double _paramInertiaIn;
  double _paramFrictionViscousIn;
  double _paramFrictionBreakIn;
  double _paramFrictionCoulombIn;
  // Friction and inertia external parameters
  double _paramInertiaOut;
  double _paramFrictionViscousOut;
  double _paramFrictionBreakOut;
  double _paramFrictionCoulombOut;
  // Electric motor parameters
  double _paramElectricVoltage;
  double _paramElectricKe;
  double _paramElectricResistance;
  // Control parameters
  double _paramControlGainP;
  double _paramControlDiscretization;
  // All inclusive time lag
  double _paramControlLag;
  // Backlash hysteresis parameters
  double _paramBacklashThresholdDeactivation;
  double _paramBacklashThresholdActivation;
  double _paramBacklashRangeMax;

  /**
   * Compute the friction force for given
   * velocity and optional given torque.
   * Internal and/or external gearbox friction
   * is used whenether isInFriction and isOutFriction
   * are set.
   */
  double computeFrictionTorque(double vel, double* torque, bool isInFriction, bool isOutFriction) const;

  /**
   * Compute the control torque
   * (without backlash model) from given
   * current position and velocity
   */
  double computeControlTorque(double pos, double vel) const;
};

}  // namespace Leph

#endif
