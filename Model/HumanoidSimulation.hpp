#ifndef LEPH_FORWARDHUMANOIDSIMULATION_HPP
#define LEPH_FORWARDHUMANOIDSIMULATION_HPP

#include <map>
#include <string>
#include <Eigen/Dense>
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Plot/Plot.hpp"

namespace Leph
{
/**
 * HumanoidSimulation
 *
 * Special implementation using generic
 * ForwardSimulation for Humanoid models
 * (Sigmaban, Grosban).
 * Handle the foot cleats contact and
 * simulation valid state.
 */
class HumanoidSimulation
{
public:
  /**
   * Initialization with robot type.
   * If inertia data and name are not empty,
   * given inertia override default model data.
   * If geometry data and name are not empty,
   * given geometry override default model data.
   */
  HumanoidSimulation(RobotType type, const Eigen::MatrixXd& inertiaData = Eigen::MatrixXd(),
                     const std::map<std::string, size_t>& inertiaName = {},
                     const Eigen::MatrixXd& geometryData = Eigen::MatrixXd(),
                     const std::map<std::string, size_t>& geometryName = {});

  /**
   * Access to internal simulated model
   */
  const HumanoidModel& model() const;
  HumanoidModel& model();

  /**
   * Set the floating base (trunk) orientation
   * to set the lowest foot (or given foot) flat
   * on the ground and aligned in yaw with the origin.
   */
  void putOnGround();
  void putOnGround(HumanoidFixedModel::SupportFoot foot);

  /**
   * Set the floating base translation (trunk)
   * to set the lowest foot (or given foot)
   * at given position.
   * If isContactFixed is true, the given foot
   * is assumed fixed on the ground and contacts
   * are not updated.
   */
  void putFootAt(double x, double y);
  void putFootAt(double x, double y, HumanoidFixedModel::SupportFoot foot, bool isContactFixed = false);

  /**
   * Internal state access
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
   * Set a target goal or position to given DOF name
   */
  void setGoal(const std::string& name, double pos);
  void setPos(const std::string& name, double pos);
  void setVel(const std::string& name, double pos);

  /**
   * Return state for given DOF name
   */
  double getGoal(const std::string& name) const;
  double getPos(const std::string& name) const;
  double getVel(const std::string& name) const;

  /**
   * Direct access to internal joint model
   */
  const JointModel& jointModel(const std::string& name) const;
  JointModel& jointModel(const std::string& name);

  /**
   * Return the vertical force appling
   * on given cleat name
   */
  double getCleatForce(const std::string& name) const;

  /**
   * Assign given joints parameters to all
   * internal joint models
   */
  void setJointModelParameters(const Eigen::VectorXd& params);

  /**
   * Run the forward simulation by one step
   * of given time duration. Handle contact
   * and collision constraints.
   */
  void update(double dt);

  /**
   * Display on standart output debuging
   * info on cleats status
   */
  void printCleatsStatus(bool verbose = true);
  void printCleatsStatus(Plot& plot);

private:
  /**
   * Structure for cleat constraint state
   */
  struct CleatState
  {
    // Model frame name
    std::string frame;
    // RBDL body id
    size_t bodyId;
    // Cleat location
    // Left or right foot,
    // top or bottom side (X),
    // left or right side (Y)
    bool isLeftFoot;
    bool isTopSide;
    bool isLeftSide;
    // Is currently in active constraints set
    bool isActive;
    // Is currently on the ground
    bool isContact;
    // If active, index of Z in constraints set
    size_t index;
    // Vertical Z force.
    // Non zero if the active
    double force;
    // Temporary cleat height
    // for constraints management
    double height;
  };

  /**
   * Robot Humanoid model
   */
  HumanoidModel _model;

  /**
   * Internal ForwardSimulation
   */
  ForwardSimulation _simulation;

  /**
   * Is the ConstraintSet initialized
   */
  bool _isInitialized;

  /**
   * RBDL current constraint set instance
   */
  RBDL::ConstraintSet _constraints;

  /**
   * All fot cleats current state container
   * indexed by cleat name
   */
  std::map<std::string, CleatState> _cleats;

  /**
   * If true, the contact are assumed to
   * be fixed and are not updated
   */
  bool _isFixedContact;

  /**
   * Add and configure a cleat with given name
   */
  void addCleat(const std::string& frame);

  /**
   * Build and return a RBDL ConstraintSet
   * with all vertical (Z) and active contact.
   * If withContact is true, also
   * add contacting cleat.
   * If withLateral is true, also
   * add lateral (X,Y) constraints.
   * If assignCleats is true, the vertical
   * constraint index is assign to cleats.
   * If isBilateralConstraint is not null,
   * the vector is assign with boolan information
   * on the associated constraint.
   */
  RBDL::ConstraintSet buildConstraintSet(bool withContact, bool withLateral, bool assignCleats,
                                         Eigen::VectorXi* isBilateralConstraint = nullptr);

  /**
   * Update cleats state (collisions detection,
   * contact lost, negative force).
   * If given isNeedLCPUpdate is assigned to true,
   * the active ConstraintSet need to be updated.
   * If given isNeedImpulse is assigned to true,
   * the system need impulses to prevent collision.
   */
  void checkAndUpdateCleatsState(bool& isNeedLCPUpdate, bool& isNeedImpulse);

  void findActiveConstraintsLCP();
};

}  // namespace Leph

#endif
