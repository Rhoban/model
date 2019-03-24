#include "QuinticWalk.hpp"
#include "Spline/SplineContainer.hpp"
#include "Utils/AxisAngle.h"
#include "Utils/Angle.h"
#include "Utils/Euler.h"

namespace Leph
{
QuinticWalk::QuinticWalk()
  : _footstep(0.14, true)
  , _phase(0.0)
  , _params()
  , _orders(0.0, 0.0, 0.0)
  , _isEnabled(false)
  , _wasEnabled(false)
  , _isTrajsOscillating(false)
  , _trunkPosAtLast()
  , _trunkVelAtLast()
  , _trunkAccAtLast()
  , _trunkAxisPosAtLast()
  , _trunkAxisVelAtLast()
  , _trunkAxisAccAtLast()
  , _trajs()
{
  // Full walk cycle frequency
  //(in Hz, > 0)
  _params.append("freq", 1.5);
  // Length of double support phase in half cycle
  //(ratio, [0:1])
  _params.append("doubleSupportRatio", 0.2);
  // Lateral distance between the feet center
  //(in m, >= 0)
  _params.append("footDistance", 0.13);
  // Maximum flying foot height
  //(in m, >= 0)
  _params.append("footRise", 0.035);
  // Phase of flying foot apex
  //(single support cycle phase, [0:1])
  _params.append("footApexPhase", 0.5);
  // Foot X/Y overshoot in ratio of step length
  //(ratio, >= 0)
  _params.append("footOvershootRatio", 0.05);
  // Foot X/Y overshoot phase
  //(single support cycle phase, [footApexPhase:1]
  _params.append("footOvershootPhase", 0.85);
  // Height of the trunk from ground
  //(in m, > 0)
  _params.append("trunkHeight", 0.27);
  // Trunk pitch orientation
  //(in rad)
  _params.append("trunkPitch", 0.2);
  // Phase offset of trunk oscillation
  //(half cycle phase, [0:1])
  _params.append("trunkPhase", 0.4);
  // Trunk forward offset
  //(in m)
  _params.append("trunkXOffset", 0.005);
  // Trunk lateral offset
  //(in m)
  _params.append("trunkYOffset", 0.0);
  // Trunk lateral oscillation amplitude ratio
  //(ratio, >= 0)
  _params.append("trunkSwing", 0.3);
  // Trunk swing pause length in phase at apex
  //(half cycle ratio, [0:1])
  _params.append("trunkPause", 0.0);
  // Trunk forward offset proportional to forward step
  //(in 1)
  _params.append("trunkXOffsetPCoefForward", 0.0);
  // Trunk forward offset proportional to rotation step
  //(in m/rad)
  _params.append("trunkXOffsetPCoefTurn", 0.0);
  // Trunk pitch orientation proportional to forward step
  //(in rad/m)
  _params.append("trunkPitchPCoefForward", 0.0);
  // Trunk pitch orientation proportional to rotation step
  //(in 1)
  _params.append("trunkPitchPCoefTurn", 0.0);

  // Initialize the footstep
  _footstep.setFootDistance(_params("footDistance"));
  _footstep.reset(true);
  // Reset the trunk saved state
  resetTrunkLastState();
  // Trajectories initialization
  buildTrajectories();
}

double QuinticWalk::getPhase() const
{
  return _phase;
}

double QuinticWalk::getTrajsTime() const
{
  double t;
  if (_phase < 0.5)
  {
    t = _phase / _params("freq");
  }
  else
  {
    t = (_phase - 0.5) / _params("freq");
  }

  return t;
}

const VectorLabel& QuinticWalk::getParameters() const
{
  return _params;
}

const Eigen::Vector3d& QuinticWalk::getOrders() const
{
  return _orders;
}

bool QuinticWalk::isEnabled() const
{
  return _isTrajsOscillating;
}

void QuinticWalk::setParameters(const VectorLabel& params)
{
  _params = params;
  _footstep.setFootDistance(_params("footDistance"));
}

void QuinticWalk::forceRebuildTrajectories()
{
  // Reset the trunk saved state
  resetTrunkLastState();
  // Rebuild the trajectories
  buildTrajectories();
}

void QuinticWalk::setOrders(const Eigen::Vector3d& orders, bool isEnabled, bool beginWithLeftSupport)
{
  // Set the walk orders
  _orders = orders;
  // Set the walk enable
  bool lastEnable = _isEnabled;
  _isEnabled = isEnabled;
  // Reset the support footstep and phase
  // to specific support foot on enable
  if (isEnabled && !lastEnable)
  {
    // Reset the footsteps
    if (beginWithLeftSupport)
    {
      _phase = 0.0 + 1e-6;
      _footstep.reset(true);
    }
    else
    {
      _phase = 0.5 + 1e-6;
      _footstep.reset(false);
    }
    // Reset the trunk saved state
    // as the support foot as been updated
    resetTrunkLastState();
    // Rebuild the trajectories
    buildTrajectories();
    // Save last enabled value
    _wasEnabled = _isEnabled;
  }
}

const Trajectories& QuinticWalk::getTrajectories() const
{
  return _trajs;
}

void QuinticWalk::update(double dt)
{
  // Check for negative time step
  if (dt <= 0.0)
  {
    std::cerr << "QuinticWalk exception negative dt phase=" << _phase << " dt=" << dt << std::endl;
    throw std::logic_error("QuinticWalk negative dt: " + std::to_string(dt));
  }
  // Check for too long dt
  if (dt > 0.25 / _params("freq"))
  {
    std::cerr << "QuinticWalk error too long dt phase=" << _phase << " dt=" << dt << std::endl;
    return;
  }

  // Update the phase
  double lastPhase = _phase;
  _phase += dt * _params("freq");
  if (_phase >= 1.0)
  {
    _phase -= 1.0;
    // Bound to zero in case
    // of floating point error
    if (_phase < 0.0)
    {
      _phase = 0.0;
    }
  }

  // Detect the phase of support foot swap
  // and computed the next half cycle
  if ((lastPhase < 0.5 && _phase >= 0.5) || (lastPhase > 0.9 && _phase < 0.1))
  {
    // Evaluate current trunk state
    //(position, velocity, acceleration)
    // in next support foot frame
    double halfPeriod = 1.0 / (2.0 * _params("freq"));
    Eigen::Vector2d trunkPos(_trajs.get("trunk_pos_x").pos(halfPeriod), _trajs.get("trunk_pos_y").pos(halfPeriod));
    Eigen::Vector2d trunkVel(_trajs.get("trunk_pos_x").vel(halfPeriod), _trajs.get("trunk_pos_y").vel(halfPeriod));
    Eigen::Vector2d trunkAcc(_trajs.get("trunk_pos_x").acc(halfPeriod), _trajs.get("trunk_pos_y").acc(halfPeriod));
    // Convert in next support foot frame
    trunkPos.x() -= _footstep.getNext().x();
    trunkPos.y() -= _footstep.getNext().y();
    trunkPos = Eigen::Rotation2Dd(-_footstep.getNext().z()).toRotationMatrix() * trunkPos;
    trunkVel = Eigen::Rotation2Dd(-_footstep.getNext().z()).toRotationMatrix() * trunkVel;
    trunkAcc = Eigen::Rotation2Dd(-_footstep.getNext().z()).toRotationMatrix() * trunkAcc;
    // Save state
    _trunkPosAtLast.x() = trunkPos.x();
    _trunkPosAtLast.y() = trunkPos.y();
    _trunkVelAtLast.x() = trunkVel.x();
    _trunkVelAtLast.y() = trunkVel.y();
    _trunkAccAtLast.x() = trunkAcc.x();
    _trunkAccAtLast.y() = trunkAcc.y();
    // No transformation for height
    _trunkPosAtLast.z() = _trajs.get("trunk_pos_z").pos(halfPeriod);
    _trunkVelAtLast.z() = _trajs.get("trunk_pos_z").vel(halfPeriod);
    _trunkAccAtLast.z() = _trajs.get("trunk_pos_z").acc(halfPeriod);
    // Evaluate and save trunk orientation
    // in next support foot frame
    Eigen::Vector3d trunkAxis(_trajs.get("trunk_axis_x").pos(halfPeriod), _trajs.get("trunk_axis_y").pos(halfPeriod),
                              _trajs.get("trunk_axis_z").pos(halfPeriod));
    // Convert in intrinsic euler angle
    Eigen::Matrix3d trunkMat = AxisToMatrix(trunkAxis);
    Eigen::Vector3d trunkEuler = MatrixToEulerIntrinsic(trunkMat);
    // Transform to next support foot
    trunkEuler.z() -= _footstep.getNext().z();
    // Reconvert to axis and save it
    trunkMat = EulerIntrinsicToMatrix(trunkEuler);
    trunkAxis = MatrixToAxis(trunkMat);
    _trunkAxisPosAtLast = trunkAxis;
    // Evaluate trunk orientation velocity
    // and acceleration without frame
    // transformation
    _trunkAxisVelAtLast.x() = _trajs.get("trunk_axis_x").vel(halfPeriod);
    _trunkAxisVelAtLast.y() = _trajs.get("trunk_axis_y").vel(halfPeriod);
    _trunkAxisVelAtLast.z() = _trajs.get("trunk_axis_z").vel(halfPeriod);
    _trunkAxisAccAtLast.x() = _trajs.get("trunk_axis_x").acc(halfPeriod);
    _trunkAxisAccAtLast.y() = _trajs.get("trunk_axis_y").acc(halfPeriod);
    _trunkAxisAccAtLast.z() = _trajs.get("trunk_axis_z").acc(halfPeriod);
    // Update footstep
    if (_isEnabled)
    {
      _footstep.stepFromOrders(_orders);
    }
    else
    {
      _footstep.stepFromOrders(Eigen::Vector3d(0.0, 0.0, 0.0));
    }
    // Build trajectories
    buildTrajectories();
    // Save last enabled value
    _wasEnabled = _isEnabled;
  }

  // Check support foot state
  if ((_phase < 0.5 && !_footstep.isLeftSupport()) || (_phase >= 0.5 && _footstep.isLeftSupport()))
  {
    std::cerr << "QuinticWalk exception invalid state phase=" << _phase << " support=" << _footstep.isLeftSupport()
              << " dt=" << dt << std::endl;
    throw std::logic_error("QuinticWalk invalid support state");
  }
}

bool QuinticWalk::assignModel(HumanoidFixedModel& model)
{
  // Compute trajectories time
  double t = getTrajsTime();

  // Evaluate target cartesian
  // state from trajectories
  Eigen::Vector3d trunkPos;
  Eigen::Vector3d trunkAxis;
  Eigen::Vector3d footPos;
  Eigen::Vector3d footAxis;
  bool isDoubleSupport;
  HumanoidFixedModel::SupportFoot supportFoot;
  TrajectoriesTrunkFootPos(t, _trajs, trunkPos, trunkAxis, footPos, footAxis);
  TrajectoriesSupportFootState(t, _trajs, isDoubleSupport, supportFoot);

  // Compute DOF positions through
  // inverse kinematics
  bool isSuccess = model.trunkFootIK(supportFoot, trunkPos, AxisToMatrix(trunkAxis), footPos, AxisToMatrix(footAxis));

  return isSuccess;
}

void QuinticWalk::buildTrajectories()
{
  // Reset the trajectories
  _trajs = TrajectoriesInit();

  // Set up the trajectories
  // for the half cycle
  double halfPeriod = 1.0 / (2.0 * _params("freq"));
  double period = 2.0 * halfPeriod;

  // Time length of double and single
  // support phase during the half cycle
  double doubleSupportLength = _params("doubleSupportRatio") * halfPeriod;
  double singleSupportLength = halfPeriod - doubleSupportLength;

  // Sign of support foot with
  // respect to lateral
  double supportSign = (_footstep.isLeftSupport() ? 1.0 : -1.0);

  // Walk disable special case
  if (!_isEnabled)
  {
    // Set double support phase
    double isDoubleSupport = (_wasEnabled ? 0.0 : 1.0);
    _trajs.get("is_double_support").addPoint(0.0, isDoubleSupport);
    _trajs.get("is_double_support").addPoint(halfPeriod, isDoubleSupport);
    // Set support foot
    _trajs.get("is_left_support_foot").addPoint(0.0, _footstep.isLeftSupport());
    _trajs.get("is_left_support_foot").addPoint(halfPeriod, _footstep.isLeftSupport());
    // Flying foot position
    _trajs.get("foot_pos_x").addPoint(0.0, _footstep.getLast().x());
    _trajs.get("foot_pos_x").addPoint(doubleSupportLength, _footstep.getLast().x());
    _trajs.get("foot_pos_x")
        .addPoint(doubleSupportLength + singleSupportLength * _params("footOvershootPhase"),
                  0.0 + (0.0 - _footstep.getLast().x()) * _params("footOvershootRatio"));
    _trajs.get("foot_pos_x").addPoint(halfPeriod, 0.0);
    _trajs.get("foot_pos_y").addPoint(0.0, _footstep.getLast().y());
    _trajs.get("foot_pos_y").addPoint(doubleSupportLength, _footstep.getLast().y());
    _trajs.get("foot_pos_y")
        .addPoint(doubleSupportLength + singleSupportLength * _params("footOvershootPhase"),
                  -supportSign * _params("footDistance") +
                      (-supportSign * _params("footDistance") - _footstep.getLast().y()) * _params("footOvershootRati"
                                                                                                   "o"));
    _trajs.get("foot_pos_y").addPoint(halfPeriod, -supportSign * _params("footDistance"));
    // If the walk has just been disabled,
    // make one single step to neutral pose
    if (_wasEnabled)
    {
      _trajs.get("foot_pos_z").addPoint(0.0, 0.0);
      _trajs.get("foot_pos_z").addPoint(doubleSupportLength, 0.0);
      _trajs.get("foot_pos_z")
          .addPoint(doubleSupportLength + singleSupportLength * _params("footApexPhase"), _params("footRise"));
      _trajs.get("foot_pos_z").addPoint(halfPeriod, 0.0);
      _isTrajsOscillating = true;
    }
    else
    {
      _trajs.get("foot_pos_z").addPoint(0.0, 0.0);
      _trajs.get("foot_pos_z").addPoint(halfPeriod, 0.0);
      _isTrajsOscillating = false;
    }
    // Flying foot orientation
    _trajs.get("foot_axis_x").addPoint(0.0, 0.0);
    _trajs.get("foot_axis_x").addPoint(halfPeriod, 0.0);
    _trajs.get("foot_axis_y").addPoint(0.0, 0.0);
    _trajs.get("foot_axis_y").addPoint(halfPeriod, 0.0);
    _trajs.get("foot_axis_z").addPoint(0.0, _footstep.getLast().z());
    _trajs.get("foot_axis_z").addPoint(doubleSupportLength, _footstep.getLast().z());
    _trajs.get("foot_axis_z").addPoint(halfPeriod, 0.0);
    // Trunk position
    _trajs.get("trunk_pos_x").addPoint(0.0, _trunkPosAtLast.x(), _trunkVelAtLast.x(), _trunkAccAtLast.x());
    _trajs.get("trunk_pos_x").addPoint(halfPeriod, _params("trunkXOffset"));
    _trajs.get("trunk_pos_y").addPoint(0.0, _trunkPosAtLast.y(), _trunkVelAtLast.y(), _trunkAccAtLast.y());
    _trajs.get("trunk_pos_y")
        .addPoint(halfPeriod, -supportSign * 0.5 * _params("footDistance") + _params("trunkYOffset"));
    _trajs.get("trunk_pos_z").addPoint(0.0, _trunkPosAtLast.z(), _trunkVelAtLast.z(), _trunkAccAtLast.z());
    _trajs.get("trunk_pos_z").addPoint(halfPeriod, _params("trunkHeight"));
    // Trunk orientation
    _trajs.get("trunk_axis_x").addPoint(0.0, _trunkAxisPosAtLast.x(), _trunkAxisVelAtLast.x(), _trunkAxisAccAtLast.x());
    _trajs.get("trunk_axis_x").addPoint(halfPeriod, 0.0);
    _trajs.get("trunk_axis_y").addPoint(0.0, _trunkAxisPosAtLast.y(), _trunkAxisVelAtLast.y(), _trunkAxisAccAtLast.y());
    _trajs.get("trunk_axis_y").addPoint(halfPeriod, _params("trunkPitch"));
    _trajs.get("trunk_axis_z").addPoint(0.0, _trunkAxisPosAtLast.z(), _trunkAxisVelAtLast.z(), _trunkAxisAccAtLast.z());
    _trajs.get("trunk_axis_z").addPoint(halfPeriod, 0.0);
    return;
  }

  // Only move the trunk on the first
  // half cycle after a walk enable
  if (_isEnabled && !_wasEnabled)
  {
    doubleSupportLength = halfPeriod;
    singleSupportLength = 0.0;
  }
  _isTrajsOscillating = true;

  // Set double support phase
  _trajs.get("is_double_support").addPoint(0.0, 0.0);
  _trajs.get("is_double_support").addPoint(halfPeriod, 0.0);
  // Set support foot
  _trajs.get("is_left_support_foot").addPoint(0.0, _footstep.isLeftSupport());
  _trajs.get("is_left_support_foot").addPoint(halfPeriod, _footstep.isLeftSupport());

  // Flying foot position
  _trajs.get("foot_pos_x").addPoint(0.0, _footstep.getLast().x());
  _trajs.get("foot_pos_x").addPoint(doubleSupportLength, _footstep.getLast().x());
  _trajs.get("foot_pos_x")
      .addPoint(doubleSupportLength + singleSupportLength * _params("footOvershootPhase"),
                _footstep.getNext().x() + (_footstep.getNext().x() - _footstep.getLast().x()) * _params("footOvershootR"
                                                                                                        "atio"));
  _trajs.get("foot_pos_x").addPoint(halfPeriod, _footstep.getNext().x());
  _trajs.get("foot_pos_y").addPoint(0.0, _footstep.getLast().y());
  _trajs.get("foot_pos_y").addPoint(doubleSupportLength, _footstep.getLast().y());
  _trajs.get("foot_pos_y")
      .addPoint(doubleSupportLength + singleSupportLength * _params("footOvershootPhase"),
                _footstep.getNext().y() + (_footstep.getNext().y() - _footstep.getLast().y()) * _params("footOvershootR"
                                                                                                        "atio"));
  _trajs.get("foot_pos_y").addPoint(halfPeriod, _footstep.getNext().y());
  _trajs.get("foot_pos_z").addPoint(0.0, 0.0);
  _trajs.get("foot_pos_z").addPoint(doubleSupportLength, 0.0);
  _trajs.get("foot_pos_z")
      .addPoint(doubleSupportLength + singleSupportLength * _params("footApexPhase"), _params("footRise"));
  _trajs.get("foot_pos_z").addPoint(halfPeriod, 0.0);

  // Flying foot orientation
  _trajs.get("foot_axis_x").addPoint(0.0, 0.0);
  _trajs.get("foot_axis_x").addPoint(halfPeriod, 0.0);
  _trajs.get("foot_axis_y").addPoint(0.0, 0.0);
  _trajs.get("foot_axis_y").addPoint(halfPeriod, 0.0);
  _trajs.get("foot_axis_z").addPoint(0.0, _footstep.getLast().z());
  _trajs.get("foot_axis_z").addPoint(doubleSupportLength, _footstep.getLast().z());
  _trajs.get("foot_axis_z").addPoint(halfPeriod, _footstep.getNext().z());

  // The trunk trajectory is defined for a
  // complete cycle to handle trunk phase shift
  // Trunk phase shift.
  double timeShift = -_params("trunkPhase") * halfPeriod;

  // Half pause length of trunk swing
  // lateral oscillation
  double pauseLength = 0.5 * _params("trunkPause") * halfPeriod;

  // Trunk support foot and next
  // support foot external
  // oscillating position
  Eigen::Vector2d trunkPointSupport(_params("trunkXOffset") +
                                        _params("trunkXOffsetPCoefForward") * _footstep.getNext().x() +
                                        _params("trunkXOffsetPCoefTurn") * std::fabs(_footstep.getNext().z()),
                                    _params("trunkYOffset"));
  Eigen::Vector2d trunkPointNext(_footstep.getNext().x() + _params("trunkXOffset") +
                                     _params("trunkXOffsetPCoefForward") * _footstep.getNext().x() +
                                     _params("trunkXOffsetPCoefTurn") * std::fabs(_footstep.getNext().z()),
                                 _footstep.getNext().y() + _params("trunkYOffset"));
  // Trunk middle neutral (no swing) position
  Eigen::Vector2d trunkPointMiddle = 0.5 * trunkPointSupport + 0.5 * trunkPointNext;
  // Trunk vector from middle to support apex
  Eigen::Vector2d trunkVect = trunkPointSupport - trunkPointMiddle;
  // Apply swing amplitude ratio
  trunkVect.y() *= _params("trunkSwing");
  // Trunk support and next apex position
  Eigen::Vector2d trunkApexSupport = trunkPointMiddle + trunkVect;
  Eigen::Vector2d trunkApexNext = trunkPointMiddle - trunkVect;
  // Trunk forward velocity
  double trunkVelSupport = (_footstep.getNext().x() - _footstep.getLast().x()) / period;
  double trunkVelNext = _footstep.getNext().x() / halfPeriod;

  // Trunk position
  _trajs.get("trunk_pos_x").addPoint(0.0, _trunkPosAtLast.x(), _trunkVelAtLast.x(), _trunkAccAtLast.x());
  _trajs.get("trunk_pos_x").addPoint(halfPeriod + timeShift, trunkApexSupport.x(), trunkVelSupport);
  _trajs.get("trunk_pos_x").addPoint(period + timeShift, trunkApexNext.x(), trunkVelNext);
  _trajs.get("trunk_pos_y").addPoint(0.0, _trunkPosAtLast.y(), _trunkVelAtLast.y(), _trunkAccAtLast.y());
  _trajs.get("trunk_pos_y").addPoint(halfPeriod + timeShift - pauseLength, trunkApexSupport.y());
  _trajs.get("trunk_pos_y").addPoint(halfPeriod + timeShift + pauseLength, trunkApexSupport.y());
  _trajs.get("trunk_pos_y").addPoint(period + timeShift - pauseLength, trunkApexNext.y());
  _trajs.get("trunk_pos_y").addPoint(period + timeShift + pauseLength, trunkApexNext.y());
  _trajs.get("trunk_pos_z").addPoint(0.0, _trunkPosAtLast.z(), _trunkVelAtLast.z(), _trunkAccAtLast.z());
  _trajs.get("trunk_pos_z").addPoint(halfPeriod + timeShift, _params("trunkHeight"));
  _trajs.get("trunk_pos_z").addPoint(period + timeShift, _params("trunkHeight"));

  // Define trunk yaw target
  // orientation position and velocity
  // in euler angle and convertion
  // to axis vector
  Eigen::Vector3d eulerAtSuport(0.0,
                                _params("trunkPitch") + _params("trunkPitchPCoefForward") * _footstep.getNext().x() +
                                    _params("trunkPitchPCoefTurn") * std::fabs(_footstep.getNext().z()),
                                0.5 * _footstep.getLast().z() + 0.5 * _footstep.getNext().z());
  Eigen::Vector3d eulerAtNext(0.0,
                              _params("trunkPitch") + _params("trunkPitchPCoefForward") * _footstep.getNext().x() +
                                  _params("trunkPitchPCoefTurn") * std::fabs(_footstep.getNext().z()),
                              _footstep.getNext().z());
  Eigen::Matrix3d matAtSupport = EulerIntrinsicToMatrix(eulerAtSuport);
  Eigen::Matrix3d matAtNext = EulerIntrinsicToMatrix(eulerAtNext);
  Eigen::Vector3d axisAtSupport = MatrixToAxis(matAtSupport);
  Eigen::Vector3d axisAtNext = MatrixToAxis(matAtNext);
  Eigen::Vector3d axisVel(0.0, 0.0, AngleDistance(_footstep.getLast().z(), _footstep.getNext().z()) / period);

  // Trunk orientation
  _trajs.get("trunk_axis_x").addPoint(0.0, _trunkAxisPosAtLast.x(), _trunkAxisVelAtLast.x(), _trunkAxisAccAtLast.x());
  _trajs.get("trunk_axis_x").addPoint(halfPeriod + timeShift, axisAtSupport.x(), axisVel.x());
  _trajs.get("trunk_axis_x").addPoint(period + timeShift, axisAtNext.x(), axisVel.x());
  _trajs.get("trunk_axis_y").addPoint(0.0, _trunkAxisPosAtLast.y(), _trunkAxisVelAtLast.y(), _trunkAxisAccAtLast.y());
  _trajs.get("trunk_axis_y").addPoint(halfPeriod + timeShift, axisAtSupport.y(), axisVel.y());
  _trajs.get("trunk_axis_y").addPoint(period + timeShift, axisAtNext.y(), axisVel.y());
  _trajs.get("trunk_axis_z").addPoint(0.0, _trunkAxisPosAtLast.z(), _trunkAxisVelAtLast.z(), _trunkAxisAccAtLast.z());
  _trajs.get("trunk_axis_z").addPoint(halfPeriod + timeShift, axisAtSupport.z(), axisVel.z());
  _trajs.get("trunk_axis_z").addPoint(period + timeShift, axisAtNext.z(), axisVel.z());
}

void QuinticWalk::resetTrunkLastState()
{
  if (_footstep.isLeftSupport())
  {
    _trunkPosAtLast << _params("trunkXOffset"), -_params("footDistance") / 2.0 + _params("trunkYOffset"),
        _params("trunkHeight");
  }
  else
  {
    _trunkPosAtLast << _params("trunkXOffset"), _params("footDistance") / 2.0 + _params("trunkYOffset"),
        _params("trunkHeight");
  }
  _trunkVelAtLast.setZero();
  _trunkAccAtLast.setZero();
  _trunkAxisPosAtLast << 0.0, _params("trunkPitch"), 0.0;
  _trunkAxisVelAtLast.setZero();
  _trunkAxisAccAtLast.setZero();
}

}  // namespace Leph
