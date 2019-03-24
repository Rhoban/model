#ifndef LEPH_FORWARDSIMULATION_H
#define LEPH_FORWARDSIMULATION_H

#include <vector>
#include <Eigen/Dense>
#include "Model/Model.hpp"
#include "Model/JointModel.hpp"

namespace Leph
{
/**
 * ForwardSimulation
 *
 * Model simulation using forward
 * dynamics and classic Runge Kutta
 * integration.
 */
class ForwardSimulation
{
public:
  /**
   * Initialization with Model instance.
   * The given model is used for kinematcis
   * and dynamics data.
   */
  ForwardSimulation(Model& model);

  /**
   * Access to given joint model by its
   * index or name
   */
  const JointModel& jointModel(size_t index) const;
  JointModel& jointModel(size_t index);
  const JointModel& jointModel(const std::string& name) const;
  JointModel& jointModel(const std::string& name);

  /**
   * Assign given joints parameters to all
   * internal joint models
   */
  void setJointModelParameters(const Eigen::VectorXd& params);

  /**
   * Access to current state
   */
  const Eigen::VectorXd& positions() const;
  Eigen::VectorXd& positions();
  const Eigen::VectorXd& velocities() const;
  Eigen::VectorXd& velocities();
  const Eigen::VectorXd& goals() const;
  Eigen::VectorXd& goals();
  const Eigen::VectorXd& jointTorques() const;
  const Eigen::VectorXd& frictionTorques() const;
  const Eigen::VectorXd& controlTorques() const;
  const Eigen::VectorXd& accelerations() const;

  /**
   * Update the position/velocity state
   * from current goal position over
   * the given dt time step.
   * Underlying model position is updated.
   * Optionnaly use given RBDL contact
   * constraint set.
   */
  void update(double dt, RBDL::ConstraintSet* constraints = nullptr);

  /**
   * Run impulse calculations and update
   * the velocities to comply with given
   * constraints set
   */
  void computeImpulses(RBDL::ConstraintSet& constraints);

  /**
   * Resolve active constraints from current position,
   * velocity and torque using Moby-Drake LCP solver
   * on given ConstraintSet.
   * isBilateralConstraint provides for each constraints
   * a boolean (non zero) if the constraint
   * is an equality (no-slip infinite friction).
   * The computed contact force lambda (with zero
   * and non zero elements) is assigned in force
   * field of constraint set.
   */
  void computeContactLCP(RBDL::ConstraintSet& constraints, const Eigen::VectorXi& isBilateralConstraint);

private:
  /**
   * Model pointer instance
   */
  Model* _model;

  /**
   * Joint model for all degrees of freedom.
   * and boolean indicating if the joint
   * is actuated. If not, the JointModel
   * is dummy.
   */
  std::vector<JointModel> _jointModels;
  std::vector<bool> _isJointActuated;

  /**
   * Degrees of freedom position
   * and velocity state
   */
  Eigen::VectorXd _positions;
  Eigen::VectorXd _velocities;

  /**
   * Degrees of freedom current
   * goal position
   */
  Eigen::VectorXd _goals;

  /**
   * Last computed accelerations, last
   * generated joint torque (decomposed in
   * friction and control) for each DOF
   */
  Eigen::VectorXd _accelerations;
  Eigen::VectorXd _jointTorques;
  Eigen::VectorXd _frictionTorques;
  Eigen::VectorXd _controlTorques;

  /**
   * Joint internal inertia offset added to
   * the diagonal of inertia matrix
   */
  Eigen::VectorXd _inertiaOffsets;
};

}  // namespace Leph

#endif
