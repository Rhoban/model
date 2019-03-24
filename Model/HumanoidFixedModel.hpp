#ifndef LEPH_HUMANOIDFIXEDMODEL_HPP
#define LEPH_HUMANOIDFIXEDMODEL_HPP

#include "Model/HumanoidModel.hpp"

namespace Leph
{
/**
 * HumanoidFixedModel
 *
 * Contains two HumanoidModel with
 * root located at each leg tip and
 * switching between the two according to
 * supporting leg.
 */
class HumanoidFixedModel
{
public:
  /**
   * Enum for support foot
   */
  enum SupportFoot
  {
    LeftSupportFoot,
    RightSupportFoot,
  };

  /**
   * Initialization with given model type.
   * If inertia data and name are not empty,
   * given inertia override default model data.
   * If geometry data and name are not empty,
   * given geometry override default model data.
   */
  HumanoidFixedModel(RobotType type, const Eigen::MatrixXd& inertiaData = Eigen::MatrixXd(),
                     const std::map<std::string, size_t>& inertiaName = {},
                     const Eigen::MatrixXd& geometryData = Eigen::MatrixXd(),
                     const std::map<std::string, size_t>& geometryName = {});

  /**
   * Virtual destructor
   */
  virtual ~HumanoidFixedModel();

  /**
   * Return the current support foot
   */
  SupportFoot getSupportFoot() const;

  /**
   * Update the current support to given
   * support foot and update odometry
   */
  void setSupportFoot(SupportFoot foot);

  /**
   * Compute using given support foot
   * orientation the robot base yaw
   * or set manually the trunk yaw
   * (a correction is applied to compute
   * the base yaw)
   */
  void setYaw(SupportFoot foot);
  void setYaw(SupportFoot foot, double trunkYaw);
  void setYaw(double trunkYaw);

  /**
   * Return Leph::HumanoidModel fixed
   * on current supporting foot tip
   */
  const HumanoidModel& get() const;
  HumanoidModel& get();

  /**
   * Update current support foot
   * and compute floating base transformation
   * to integrate model displacement
   */
  virtual void updateBase();

  /**
   * Update support foot floating base
   * (complete rotation matrix)
   * orientation in order that trunk
   * orientation matches given orientation (IMU)
   * in origin.
   * If applyYaw is false, the base_yaw DOF
   * is not assign.
   */
  void setOrientation(const Eigen::Matrix3d& originToTrunk, bool applyYaw = true);

  /**
   * Update base_x and base_y DOF in order
   * that trunk x/y position in world frame
   * matches given planar position.
   * Use to assign and override corrected odometry.
   */
  void setOdometryState(const Eigen::Vector2d& pose);

  /**
   * Compute and return the Zero Moment Point
   * (ZMP) in given frame. Given degrees of freedom
   * velocity and acceleration are used (to compute
   * inverse dynamics).
   * Current degrees of freedom position
   * and support foot are used.
   * If isDoubleSupport is true, the ZMP is computed
   * for double support case assuming given velocity
   * and acceleration comply with closed loop constraints.
   */
  Eigen::Vector3d zeroMomentPoint(const std::string& frame, const Eigen::VectorXd& velocity,
                                  const Eigen::VectorXd& acceleration, bool isDoubleSupport = false);

  /**
   * Compute the zero moment point in given frame
   * either from single support or double support torques.
   * Current degrees of freedom position and
   * support foot are used.
   * Single or double support torque from inverseDynamics.
   * The contact force is given in flying foot frame for
   * double support calculation.
   */
  Eigen::Vector3d zeroMomentPointSingleSupport(const std::string& frame, const Eigen::VectorXd& torques);
  Eigen::Vector3d zeroMomentPointDoubleSupport(const std::string& frame, const Eigen::VectorXd& torques,
                                               const Eigen::VectorXd& contactForces);

  /**
   * Set the model state by running
   * Inverse Kinematics on both legs.
   * The state is given by the support foot
   * (left ot right), the flying foot position
   * and orientation, the position
   * and orientation of the trunk with
   * respect to support foot.
   * Support foot is set flat on the ground.
   * False is returned if the inverse
   * kinematics fails.
   * If not null, boundIKDistance is a signed "distance"
   * from kinematics bound. If positive, the IK is valid.
   * If negative, the IK is out of bounds.
   */
  bool trunkFootIK(SupportFoot support, const Eigen::Vector3d& trunkPos, const Eigen::Matrix3d& trunkRotation,
                   const Eigen::Vector3d& flyingFootPos, const Eigen::Matrix3d& flyingFootRotation,
                   double* boundIKDistance = nullptr);

  /**
   * Compute and return all degrees of
   * freedom velocities from given
   * trunk translation, orientation velocities
   * and flying foot translation, orientation
   * velocity in support foot frame.
   * Translation velocity is the velocity vector.
   * Orientation velocity is the instantaneous
   * angular velocity vector (in support foot frame)
   * !!!
   * !!! Angular velocity vector is not the
   * !!! differentiation of axis angle representation
   * !!! (see AxisAngle::AxisDiffToAxisVel())
   * !!!
   * Return null velocity in case of
   * near singular jacobian
   */
  Eigen::VectorXd trunkFootIKVel(const Eigen::Vector3d& trunkPosVel, const Eigen::Vector3d& trunkAxisAnglesVel,
                                 const Eigen::Vector3d& flyingFootPosVel,
                                 const Eigen::Vector3d& flyingFootAxisAnglesVel);

  /**
   * Compute and return all degrees of freedom
   * accelerations from given trunk translation,
   * orientation and flying foot translation,
   * orientation velocities and accelerations
   * in support foot frame. All degrees of freedom
   * velocities are also given.
   * Translation velocity/acceleration
   * is the velocity/acceleration vector.
   * Orientation velocity/acceleration is the instantaneous
   * angular velocity/acceleration vector (in support foot frame)
   * !!!
   * !!! Angular velocity/acceleration vector is not the
   * !!! differentiation of axis angle representation
   * !!! (see AxisAngle::AxisDiffToAxisVel())
   * !!!
   * Return null acceleration in case of
   * near singular jacobian
   */
  Eigen::VectorXd trunkFootIKAcc(const Eigen::VectorXd& dq, const Eigen::Vector3d& trunkPosVel,
                                 const Eigen::Vector3d& trunkAxisAnglesVel, const Eigen::Vector3d& flyingFootPosVel,
                                 const Eigen::Vector3d& flyingFootAxisAnglesVel, const Eigen::Vector3d& trunkPosAcc,
                                 const Eigen::Vector3d& trunkAxisAnglesAcc, const Eigen::Vector3d& flyingFootPosAcc,
                                 const Eigen::Vector3d& flyingFootAxisAnglesAcc);

private:
  /**
   * Current support foot
   */
  SupportFoot _supportFoot;

  /**
   * HumanoidModel for left and right
   * support foot
   */
  HumanoidModel _modelLeft;
  HumanoidModel _modelRight;

  /**
   * Compute the ZMP position given linear
   * Z force and X/Y moment in local foot frame.
   * Returned ZMP is in local foot frame.
   */
  Eigen::Vector3d computeZMP(double Mx, double My, double Fz);
};

}  // namespace Leph

#endif
