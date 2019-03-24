#include "QuinticWalk/QuinticExperiment.hpp"
#include "Utils/AxisAngle.h"

namespace Leph
{
QuinticExperiment::QuinticExperiment() : _params(defaultParameters()), _trajs()
{
  _trajs = generateTrajectories();
}

void QuinticExperiment::setParameters(const Parameters& params)
{
  _params = params;
  _trajs = generateTrajectories();
}

const QuinticExperiment::Parameters QuinticExperiment::getParameters() const
{
  return _params;
}

const Trajectories& QuinticExperiment::getTrajectories() const
{
  return _trajs;
}

bool QuinticExperiment::computePose(HumanoidFixedModel& model, double phase) const
{
  // Compute cartesian target form splines
  double t = phaseToTime(phase);
  Eigen::Vector3d trunkPos;
  Eigen::Vector3d trunkAxis;
  Eigen::Vector3d footPos;
  Eigen::Vector3d footAxis;
  bool isDoubleSupport;
  HumanoidFixedModel::SupportFoot supportFoot;
  TrajectoriesTrunkFootPos(t, _trajs, trunkPos, trunkAxis, footPos, footAxis);
  TrajectoriesSupportFootState(t, _trajs, isDoubleSupport, supportFoot);
  // Compute DOF positions
  bool isSuccess = model.trunkFootIK(supportFoot, trunkPos, AxisToMatrix(trunkAxis), footPos, AxisToMatrix(footAxis));
  return isSuccess;
}

double QuinticExperiment::updatePhase(double phase, double dt) const
{
  double time = _params(0) * phase;
  time += dt;
  double phaseNext = time / _params(0);
  if (phaseNext > 1.0)
  {
    phaseNext -= 1.0;
  }

  return phaseNext;
}

double QuinticExperiment::phaseToTime(double phase) const
{
  return _params(0) * phase;
}

QuinticExperiment::Parameters QuinticExperiment::defaultParameters()
{
  Eigen::VectorXd params = Eigen::VectorXd::Zero(44);
  params << 0.587908861782481, 0.0366993217871195, 0.0374323902074378, -0.142998273687543, 0.0549407509520598,
      -0.0714700569395352, 0.293404309614006, 0.0907329120261863, -0.092261262558954, -3.41792917876447e-07,
      4.38499346000971, 2.29630388147032, 1.71037641322152, -2.48126652273306e-10, 0.149999991776317,
      -1.16047373162189e-08, -4.9128066379382e-08, -4.37503435953843e-07, 7.01008701998637e-06, 4.63190288246227,
      1.57555094018333, -5.81717909853587, 0.0360297008049997, -0.0543930630782171, 0.293404317347432,
      0.125913785907179, -1.0464933421546e-07, 3.23423437562951, -5.31053103899626, -3.23163351404329,
      4.5136184338577e-09, 0.150000005230885, -1.71309894465846e-07, -2.55441396862228e-07, -4.60480256944335e-06,
      -1.3482133702974, -1.20448302808787, 1.13880478562503, 0.0205359716550638, 0.2660737760555, -3.05712519950543e-06,
      15.2647432128294, -0.596055596325538, 15.4463810408885;

  return params;
}

Trajectories QuinticExperiment::generateTrajectories() const
{
  // Walk configuration
  double cycleLength = _params(0);
  double footHeight = _params(1);
  double footStep = _params(2);
  double footLateralOffset = _params(3);

  // Time ratio
  double timeRightBegin = 0.0;
  double timeRightApex = cycleLength / 4.0;
  double timeMiddle = cycleLength / 2.0;
  double timeLeftApex = 3.0 * cycleLength / 4.0;
  double timeLeftEnd = cycleLength;

  // Double support target state
  Eigen::Vector3d trunkPos_Middle(_params(4), _params(5), _params(6));
  Eigen::Vector3d trunkPosVel_Middle(_params(7), _params(8), _params(9));
  Eigen::Vector3d trunkPosAcc_Middle(_params(10), _params(11), _params(12));
  // XXX Eigen::Vector3d trunkPosAcc_Middle(0.0, 0.0, 0.0);
  Eigen::Vector3d trunkAxis_Middle(_params(13), _params(14), _params(15));
  Eigen::Vector3d trunkAxisVel_Middle(_params(16), _params(17), _params(18));
  Eigen::Vector3d trunkAxisAcc_Middle(_params(19), _params(20), _params(21));
  // XXX Eigen::Vector3d trunkAxisAcc_Middle(0.0, 0.0, 0.0);
  Eigen::Vector3d footPos_Middle(footStep, footLateralOffset, 0.0);

  // Single support target state
  Eigen::Vector3d trunkPos_Apex(_params(22), _params(23), _params(24));
  Eigen::Vector3d trunkPosVel_Apex(_params(25), 0.0, _params(26));
  Eigen::Vector3d trunkPosAcc_Apex(_params(27), _params(28), _params(29));
  // XXX Eigen::Vector3d trunkPosAcc_Apex(0.0, 0.0, 0.0);
  Eigen::Vector3d trunkAxis_Apex(_params(30), _params(31), _params(32));
  Eigen::Vector3d trunkAxisVel_Apex(0.0, _params(33), _params(34));
  Eigen::Vector3d trunkAxisAcc_Apex(_params(35), _params(36), _params(37));
  // XXX Eigen::Vector3d trunkAxisAcc_Apex(0.0, 0.0, 0.0);
  Eigen::Vector3d footPos_Apex(_params(38), footLateralOffset, footHeight);
  Eigen::Vector3d footPosVel_Apex(_params(39), _params(40), 0.0);
  Eigen::Vector3d footPosAcc_Apex(_params(41), _params(42), _params(43));
  // XXX Eigen::Vector3d footPosAcc_Apex(0.0, 0.0, 0.0);

  // Initialize state splines
  Trajectories traj = TrajectoriesInit();

  // Support foot phase
  traj.get("is_double_support").addPoint(timeRightBegin, 0.0);
  traj.get("is_double_support").addPoint(timeLeftEnd, 0.0);
  // Support foot
  traj.get("is_left_support_foot").addPoint(timeRightBegin, 0.0);
  traj.get("is_left_support_foot").addPoint(timeMiddle, 0.0);
  traj.get("is_left_support_foot").addPoint(timeMiddle, 1.0);
  traj.get("is_left_support_foot").addPoint(timeLeftEnd, 1.0);

  // Foot orientation
  traj.get("foot_axis_x").addPoint(timeRightBegin, 0.0);
  traj.get("foot_axis_x").addPoint(timeLeftEnd, 0.0);
  traj.get("foot_axis_y").addPoint(timeRightBegin, 0.0);
  traj.get("foot_axis_y").addPoint(timeLeftEnd, 0.0);
  traj.get("foot_axis_z").addPoint(timeRightBegin, 0.0);
  traj.get("foot_axis_z").addPoint(timeLeftEnd, 0.0);

  // Right Begin
  traj.get("trunk_pos_x")
      .addPoint(timeRightBegin, trunkPos_Middle.x() - footPos_Middle.x(), trunkPosVel_Middle.x(),
                trunkPosAcc_Middle.x());
  traj.get("trunk_pos_y")
      .addPoint(timeRightBegin, trunkPos_Middle.y() - footPos_Middle.y(), trunkPosVel_Middle.y(),
                trunkPosAcc_Middle.y());
  traj.get("trunk_pos_z").addPoint(timeRightBegin, trunkPos_Middle.z(), trunkPosVel_Middle.z(), trunkPosAcc_Middle.z());
  traj.get("trunk_axis_x")
      .addPoint(timeRightBegin, trunkAxis_Middle.x(), trunkAxisVel_Middle.x(), trunkAxisAcc_Middle.x());
  traj.get("trunk_axis_y")
      .addPoint(timeRightBegin, trunkAxis_Middle.y(), trunkAxisVel_Middle.y(), trunkAxisAcc_Middle.y());
  traj.get("trunk_axis_z")
      .addPoint(timeRightBegin, trunkAxis_Middle.z(), trunkAxisVel_Middle.z(), trunkAxisAcc_Middle.z());
  traj.get("foot_pos_x").addPoint(timeRightBegin, -footPos_Middle.x(), 0.0, 0.0);
  traj.get("foot_pos_y").addPoint(timeRightBegin, -footPos_Middle.y(), 0.0, 0.0);
  traj.get("foot_pos_z").addPoint(timeRightBegin, footPos_Middle.z(), 0.0, 0.0);

  // Right Apex
  traj.get("trunk_pos_x").addPoint(timeRightApex, trunkPos_Apex.x(), trunkPosVel_Apex.x(), trunkPosAcc_Apex.x());
  traj.get("trunk_pos_y").addPoint(timeRightApex, -trunkPos_Apex.y(), -trunkPosVel_Apex.y(), -trunkPosAcc_Apex.y());
  traj.get("trunk_pos_z").addPoint(timeRightApex, trunkPos_Apex.z(), trunkPosVel_Apex.z(), trunkPosAcc_Apex.z());
  traj.get("trunk_axis_x").addPoint(timeRightApex, -trunkAxis_Apex.x(), -trunkAxisVel_Apex.x(), -trunkAxisAcc_Apex.x());
  traj.get("trunk_axis_y").addPoint(timeRightApex, trunkAxis_Apex.y(), trunkAxisVel_Apex.y(), trunkAxisAcc_Apex.y());
  traj.get("trunk_axis_z").addPoint(timeRightApex, trunkAxis_Apex.z(), trunkAxisVel_Apex.z(), trunkAxisAcc_Apex.z());
  traj.get("foot_pos_x").addPoint(timeRightApex, footPos_Apex.x(), footPosVel_Apex.x(), footPosAcc_Apex.x());
  traj.get("foot_pos_y").addPoint(timeRightApex, -footPos_Apex.y(), -footPosVel_Apex.y(), -footPosAcc_Apex.y());
  traj.get("foot_pos_z").addPoint(timeRightApex, footPos_Apex.z(), footPosVel_Apex.z(), footPosAcc_Apex.z());

  // Middle
  traj.get("trunk_pos_x").addPoint(timeMiddle, trunkPos_Middle.x(), trunkPosVel_Middle.x(), trunkPosAcc_Middle.x());
  traj.get("trunk_pos_y").addPoint(timeMiddle, -trunkPos_Middle.y(), -trunkPosVel_Middle.y(), -trunkPosAcc_Middle.y());
  traj.get("trunk_pos_z").addPoint(timeMiddle, trunkPos_Middle.z(), trunkPosVel_Middle.z(), trunkPosAcc_Middle.z());
  traj.get("trunk_axis_x")
      .addPoint(timeMiddle, -trunkAxis_Middle.x(), -trunkAxisVel_Middle.x(), -trunkAxisAcc_Middle.x());
  traj.get("trunk_axis_y").addPoint(timeMiddle, trunkAxis_Middle.y(), trunkAxisVel_Middle.y(), trunkAxisAcc_Middle.y());
  traj.get("trunk_axis_z").addPoint(timeMiddle, trunkAxis_Middle.z(), trunkAxisVel_Middle.z(), trunkAxisAcc_Middle.z());
  traj.get("foot_pos_x").addPoint(timeMiddle, footPos_Middle.x(), 0.0, 0.0);
  traj.get("foot_pos_y").addPoint(timeMiddle, -footPos_Middle.y(), 0.0, 0.0);
  traj.get("foot_pos_z").addPoint(timeMiddle, footPos_Middle.z(), 0.0, 0.0);

  // Middle Support foot swap
  traj.get("trunk_pos_x")
      .addPoint(timeMiddle, trunkPos_Middle.x() - footPos_Middle.x(), trunkPosVel_Middle.x(), trunkPosAcc_Middle.x());
  traj.get("trunk_pos_y")
      .addPoint(timeMiddle, -trunkPos_Middle.y() + footPos_Middle.y(), -trunkPosVel_Middle.y(),
                -trunkPosAcc_Middle.y());
  traj.get("trunk_pos_z").addPoint(timeMiddle, trunkPos_Middle.z(), trunkPosVel_Middle.z(), trunkPosAcc_Middle.z());
  traj.get("trunk_axis_x")
      .addPoint(timeMiddle, -trunkAxis_Middle.x(), -trunkAxisVel_Middle.x(), -trunkAxisAcc_Middle.x());
  traj.get("trunk_axis_y").addPoint(timeMiddle, trunkAxis_Middle.y(), trunkAxisVel_Middle.y(), trunkAxisAcc_Middle.y());
  traj.get("trunk_axis_z").addPoint(timeMiddle, trunkAxis_Middle.z(), trunkAxisVel_Middle.z(), trunkAxisAcc_Middle.z());
  traj.get("foot_pos_x").addPoint(timeMiddle, -footPos_Middle.x(), 0.0, 0.0);
  traj.get("foot_pos_y").addPoint(timeMiddle, footPos_Middle.y(), 0.0, 0.0);
  traj.get("foot_pos_z").addPoint(timeMiddle, footPos_Middle.z(), 0.0, 0.0);

  // Left Apex
  traj.get("trunk_pos_x").addPoint(timeLeftApex, trunkPos_Apex.x(), trunkPosVel_Apex.x(), trunkPosAcc_Apex.x());
  traj.get("trunk_pos_y").addPoint(timeLeftApex, trunkPos_Apex.y(), trunkPosVel_Apex.y(), trunkPosAcc_Apex.y());
  traj.get("trunk_pos_z").addPoint(timeLeftApex, trunkPos_Apex.z(), trunkPosVel_Apex.z(), trunkPosAcc_Apex.z());
  traj.get("trunk_axis_x").addPoint(timeLeftApex, trunkAxis_Apex.x(), trunkAxisVel_Apex.x(), trunkAxisAcc_Apex.x());
  traj.get("trunk_axis_y").addPoint(timeLeftApex, trunkAxis_Apex.y(), trunkAxisVel_Apex.y(), trunkAxisAcc_Apex.y());
  traj.get("trunk_axis_z").addPoint(timeLeftApex, trunkAxis_Apex.z(), trunkAxisVel_Apex.z(), trunkAxisAcc_Apex.z());
  traj.get("foot_pos_x").addPoint(timeLeftApex, footPos_Apex.x(), footPosVel_Apex.x(), footPosAcc_Apex.x());
  traj.get("foot_pos_y").addPoint(timeLeftApex, footPos_Apex.y(), footPosVel_Apex.y(), footPosAcc_Apex.y());
  traj.get("foot_pos_z").addPoint(timeLeftApex, footPos_Apex.z(), footPosVel_Apex.z(), footPosAcc_Apex.z());

  // Left End
  traj.get("trunk_pos_x").addPoint(timeLeftEnd, trunkPos_Middle.x(), trunkPosVel_Middle.x(), trunkPosAcc_Middle.x());
  traj.get("trunk_pos_y").addPoint(timeLeftEnd, trunkPos_Middle.y(), trunkPosVel_Middle.y(), trunkPosAcc_Middle.y());
  traj.get("trunk_pos_z").addPoint(timeLeftEnd, trunkPos_Middle.z(), trunkPosVel_Middle.z(), trunkPosAcc_Middle.z());
  traj.get("trunk_axis_x")
      .addPoint(timeLeftEnd, trunkAxis_Middle.x(), trunkAxisVel_Middle.x(), trunkAxisAcc_Middle.x());
  traj.get("trunk_axis_y")
      .addPoint(timeLeftEnd, trunkAxis_Middle.y(), trunkAxisVel_Middle.y(), trunkAxisAcc_Middle.y());
  traj.get("trunk_axis_z")
      .addPoint(timeLeftEnd, trunkAxis_Middle.z(), trunkAxisVel_Middle.z(), trunkAxisAcc_Middle.z());
  traj.get("foot_pos_x").addPoint(timeLeftEnd, footPos_Middle.x(), 0.0, 0.0);
  traj.get("foot_pos_y").addPoint(timeLeftEnd, footPos_Middle.y(), 0.0, 0.0);
  traj.get("foot_pos_z").addPoint(timeLeftEnd, footPos_Middle.z(), 0.0, 0.0);

  return traj;
}

}  // namespace Leph
