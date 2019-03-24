#include "IKWalk/IKWalk.hpp"
#include "Spline/CubicSpline.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Utils/Euler.h"

namespace Leph
{
bool IKWalk::walk(HumanoidModel& model, const Parameters& params, double& phase, double dt)
{
  // Compute phase for left and right leg
  double phaseLeft = phase;
  double phaseRight = phase + 0.5;
  boundPhase(phaseLeft);
  boundPhase(phaseRight);

  // Compute the length of a step
  //(from ground touch to take off) in phase time
  double stepLength = 0.5 * params.supportPhaseRatio + 0.5;

  // Build X foot step spline
  // The foot goes backward between t=0 and t=stepLength and
  // then goes forward. Custom velovity (tangents) are applied at
  // foot take off and landing.
  // During foot backward movement, constant velocity is
  // apply because both foot must have the same velocity
  // during double support phase.
  CubicSpline stepSpline;
  stepSpline.addPoint(0.0, 0.5, -1.0 / stepLength);
  stepSpline.addPoint(stepLength, -0.5, -1.0 / stepLength);
  stepSpline.addPoint(stepLength, -0.5, params.stepUpVel);
  stepSpline.addPoint(1.0, 0.5, -params.stepDownVel);

  // Build Y trunk swing spline.
  // The trunk lateral oscillation goes from right to left,
  // wait a bit (swingPause) on left side then goes to the
  // right and pause as well.
  // Trajectory "smoothness" can be tunned with
  // swingVel updating splines tangents.
  CubicSpline swingSpline;
  swingSpline.addPoint(0.0, 1.0);
  swingSpline.addPoint(params.swingPause / 2.0, 1.0);
  swingSpline.addPoint(params.swingPause / 2.0, 1.0, -params.swingVel);
  swingSpline.addPoint(0.5 - params.swingPause / 2.0, -1.0, -params.swingVel);
  swingSpline.addPoint(0.5 - params.swingPause / 2.0, -1.0);
  swingSpline.addPoint(0.5 + params.swingPause / 2.0, -1.0);
  swingSpline.addPoint(0.5 + params.swingPause / 2.0, -1.0, params.swingVel);
  swingSpline.addPoint(1.0 - params.swingPause / 2.0, 1.0, params.swingVel);
  swingSpline.addPoint(1.0 - params.swingPause / 2.0, 1.0);
  swingSpline.addPoint(1.0, 1.0, 0.0);

  // Build Z foot rise spline.
  // The foot stays on the ground during backward step and then
  // moves up and down.
  // Custom velocities (tangents) can be tunned to achieve
  // specific trajectory at foot take off and landing.
  CubicSpline riseSpline;
  riseSpline.addPoint(0.0, 0.0);
  riseSpline.addPoint(stepLength, 0.0);
  riseSpline.addPoint(stepLength, 0.0, params.riseUpVel);
  riseSpline.addPoint((1.0 + stepLength) / 2.0, 1.0);
  riseSpline.addPoint(1.0, 0.0, -params.riseDownVel);

  // Build Yaw foot turn spline.
  // This is the same as stepSpline but movement occurs
  // only during single support phase as robot degrees of freedom
  // could not achieve rotation during double support phase.
  SmoothSpline turnSpline;
  turnSpline.addPoint(0.0, 0.0);
  turnSpline.addPoint(stepLength - 0.5, 0.0);
  turnSpline.addPoint(0.5, 1.0);
  turnSpline.addPoint(stepLength, 1.0);
  turnSpline.addPoint(1.0, 0.0);

  // Compute swing value
  double swingVal = params.enabledGain * params.swingGain * swingSpline.posMod(phaseLeft + params.swingPhase);

  // Compute feet forward (step) oscillation
  double leftX = params.enabledGain * params.stepGain * stepSpline.pos(phaseLeft);
  double rightX = params.enabledGain * params.stepGain * stepSpline.pos(phaseRight);

  // Compute feet swing oscillation
  double leftY = swingVal;
  double rightY = swingVal;
  // Compute feet lateral movement oscillation
  leftY += params.enabledGain * params.lateralGain *
           (stepSpline.pos(phaseLeft) + 0.5 * (params.lateralGain >= 0.0 ? 1.0 : -1.0));
  rightY += params.enabledGain * params.lateralGain *
            (stepSpline.pos(phaseRight) + 0.5 * (params.lateralGain >= 0.0 ? -1.0 : 1.0));
  // Set feet lateral offset (feet distance from trunk center)
  leftY += params.footYOffset;
  rightY += -params.footYOffset;

  // Compute feet vertical (rise) oscillation and offset
  double leftZ = params.enabledGain * params.riseGain * riseSpline.pos(phaseLeft);
  double rightZ = params.enabledGain * params.riseGain * riseSpline.pos(phaseRight);
  // Set trunk to foot distance height offset
  leftZ += params.trunkZOffset;
  rightZ += params.trunkZOffset;

  // Compute feet rotation (turn) oscillation
  double leftYaw = params.enabledGain * params.turnGain * turnSpline.pos(phaseLeft);
  double rightYaw = params.enabledGain * params.turnGain * turnSpline.pos(phaseRight);

  // Compute trunk roll angle
  double rollVal = params.enabledGain * -params.swingRollGain * swingSpline.posMod(0.5 + phaseLeft + params.swingPhase);
  // Set trunk roll offset
  rollVal += params.trunkRoll;

  // Set feet orientation
  double leftPitch = params.trunkPitch;
  double leftRoll = rollVal;
  double rightPitch = params.trunkPitch;
  double rightRoll = rollVal;

  // Add custom extra foot offset on both feet
  leftX += params.extraLeftX;
  leftY += params.extraLeftY;
  leftZ += params.extraLeftZ;
  leftYaw += params.extraLeftYaw;
  leftPitch += params.extraLeftPitch;
  leftRoll += params.extraLeftRoll;
  rightX += params.extraRightX;
  rightY += params.extraRightY;
  rightZ += params.extraRightZ;
  rightYaw += params.extraRightYaw;
  rightPitch += params.extraRightPitch;
  rightRoll += params.extraRightRoll;

  // Build rotation matrix for trunk pitch and roll
  // orientation
  Eigen::AngleAxisd pitchRot(-params.trunkPitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rollRot(-rollVal, Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> quat(pitchRot * rollRot);
  Eigen::Matrix3d rotation = quat.matrix();

  // Build target vector.
  // Used Euler angles orders is Pitch Roll Yaw because
  // Yaw has to be applied last, after the foot get the good
  // ground orientation. Roll has to be applied after Pitch.
  Eigen::Vector3d posLeft(leftX, leftY, leftZ);
  Eigen::Vector3d angleLeft(leftPitch, leftRoll, leftYaw);
  Eigen::Vector3d posRight(rightX, rightY, rightZ);
  Eigen::Vector3d angleRight(rightPitch, rightRoll, rightYaw);

  // Rotate built feet trajectory to
  // meet asked trunk Pitch and Roll new
  // ground orientation
  posLeft = rotation * posLeft;
  posRight = rotation * posRight;

  // Apply trunk X-Y offset
  posLeft(0) -= params.trunkXOffset;
  posRight(0) -= params.trunkXOffset;
  posLeft(1) -= params.trunkYOffset;
  posRight(1) -= params.trunkYOffset;

  // In case of trunk Roll rotation, an height (Z)
  // positive offset have to be applied on external foot to
  // set both feet on same level
  double deltaLen = model.feetDistance() * tan(rollVal);
  if (rollVal > 0.0)
  {
    posRight(2) += deltaLen;
  }
  else if (rollVal < 0.0)
  {
    posLeft(2) -= deltaLen;
  }
  // TODO In case of oscillating trinkRoll or swingRoll enabled XXX
  // TODO feet get an unwanted lateral oscillation XXX

  // Trunk X and Y offset is applied to compensate
  // Pitch and Roll rotation. It is better for tunning if
  // trunk pitch or roll rotation do not apply offset on
  // trunk position.
  posLeft(0) += (model.legsLength()) * tan(params.trunkPitch);
  posRight(0) += (model.legsLength()) * tan(params.trunkPitch);
  posLeft(1) -= (model.legsLength()) * tan(rollVal);
  posRight(1) -= (model.legsLength()) * tan(rollVal);

  // Run inverse invert kinematics on both legs
  // using Pitch-Roll-Yaw convention
  bool successLeft = model.legIkLeft("foot_tip_init", posLeft, EulerToMatrix(angleLeft, EulerPitchRollYaw));
  bool successRight = model.legIkRight("foot_tip_init", posRight, EulerToMatrix(angleRight, EulerPitchRollYaw));

  // Check inverse kinematics success
  if (!successLeft || !successRight)
  {
    return false;
  }

  // Increment given phase
  phase += dt * params.freq;
  // Cycling between  and 1
  boundPhase(phase);

  return true;
}

void IKWalk::convertParameters(VectorLabel& vect, const Parameters& params)
{
  vect.setOrAppend("walk:freq", params.freq);
  vect.setOrAppend("walk:enabledGain", params.enabledGain);
  vect.setOrAppend("walk:supportPhaseRatio", params.supportPhaseRatio);
  vect.setOrAppend("walk:footYOffset", params.footYOffset);
  vect.setOrAppend("walk:stepGain", params.stepGain);
  vect.setOrAppend("walk:riseGain", params.riseGain);
  vect.setOrAppend("walk:turnGain", params.turnGain);
  vect.setOrAppend("walk:lateralGain", params.lateralGain);
  vect.setOrAppend("walk:trunkZOffset", params.trunkZOffset);
  vect.setOrAppend("walk:swingGain", params.swingGain);
  vect.setOrAppend("walk:swingRollGain", params.swingRollGain);
  vect.setOrAppend("walk:swingPhase", params.swingPhase);
  vect.setOrAppend("walk:stepUpVel", params.stepUpVel);
  vect.setOrAppend("walk:stepDownVel", params.stepDownVel);
  vect.setOrAppend("walk:riseUpVel", params.riseUpVel);
  vect.setOrAppend("walk:riseDownVel", params.riseDownVel);
  vect.setOrAppend("walk:swingPause", params.swingPause);
  vect.setOrAppend("walk:swingVel", params.swingVel);
  vect.setOrAppend("walk:trunkXOffset", params.trunkXOffset);
  vect.setOrAppend("walk:trunkYOffset", params.trunkYOffset);
  vect.setOrAppend("walk:trunkPitch", params.trunkPitch);
  vect.setOrAppend("walk:trunkRoll", params.trunkRoll);
  vect.setOrAppend("walk:extraLeftX", params.extraLeftX);
  vect.setOrAppend("walk:extraLeftY", params.extraLeftY);
  vect.setOrAppend("walk:extraLeftZ", params.extraLeftZ);
  vect.setOrAppend("walk:extraRightX", params.extraRightX);
  vect.setOrAppend("walk:extraRightY", params.extraRightY);
  vect.setOrAppend("walk:extraRightZ", params.extraRightZ);
  vect.setOrAppend("walk:extraLeftYaw", params.extraLeftYaw);
  vect.setOrAppend("walk:extraLeftPitch", params.extraLeftPitch);
  vect.setOrAppend("walk:extraLeftRoll", params.extraLeftRoll);
  vect.setOrAppend("walk:extraRightYaw", params.extraRightYaw);
  vect.setOrAppend("walk:extraRightPitch", params.extraRightPitch);
  vect.setOrAppend("walk:extraRightRoll", params.extraRightRoll);
}

IKWalk::Parameters IKWalk::convertVectorLabel(const VectorLabel& vect)
{
  Parameters params;

  if (vect.exist("walk:freq"))
    params.freq = vect("walk:freq");
  if (vect.exist("walk:enabledGain"))
    params.enabledGain = vect("walk:enabledGain");
  if (vect.exist("walk:supportPhaseRatio"))
    params.supportPhaseRatio = vect("walk:supportPhaseRatio");
  if (vect.exist("walk:footYOffset"))
    params.footYOffset = vect("walk:footYOffset");
  if (vect.exist("walk:stepGain"))
    params.stepGain = vect("walk:stepGain");
  if (vect.exist("walk:riseGain"))
    params.riseGain = vect("walk:riseGain");
  if (vect.exist("walk:turnGain"))
    params.turnGain = vect("walk:turnGain");
  if (vect.exist("walk:lateralGain"))
    params.lateralGain = vect("walk:lateralGain");
  if (vect.exist("walk:trunkZOffset"))
    params.trunkZOffset = vect("walk:trunkZOffset");
  if (vect.exist("walk:swingGain"))
    params.swingGain = vect("walk:swingGain");
  if (vect.exist("walk:swingRollGain"))
    params.swingRollGain = vect("walk:swingRollGain");
  if (vect.exist("walk:swingPhase"))
    params.swingPhase = vect("walk:swingPhase");
  if (vect.exist("walk:stepUpVel"))
    params.stepUpVel = vect("walk:stepUpVel");
  if (vect.exist("walk:stepDownVel"))
    params.stepDownVel = vect("walk:stepDownVel");
  if (vect.exist("walk:riseUpVel"))
    params.riseUpVel = vect("walk:riseUpVel");
  if (vect.exist("walk:riseDownVel"))
    params.riseDownVel = vect("walk:riseDownVel");
  if (vect.exist("walk:swingPause"))
    params.swingPause = vect("walk:swingPause");
  if (vect.exist("walk:swingVel"))
    params.swingVel = vect("walk:swingVel");
  if (vect.exist("walk:trunkXOffset"))
    params.trunkXOffset = vect("walk:trunkXOffset");
  if (vect.exist("walk:trunkYOffset"))
    params.trunkYOffset = vect("walk:trunkYOffset");
  if (vect.exist("walk:trunkPitch"))
    params.trunkPitch = vect("walk:trunkPitch");
  if (vect.exist("walk:trunkRoll"))
    params.trunkRoll = vect("walk:trunkRoll");
  if (vect.exist("walk:extraLeftX"))
    params.extraLeftX = vect("walk:extraLeftX");
  if (vect.exist("walk:extraLeftY"))
    params.extraLeftY = vect("walk:extraLeftY");
  if (vect.exist("walk:extraLeftZ"))
    params.extraLeftZ = vect("walk:extraLeftZ");
  if (vect.exist("walk:extraRightX"))
    params.extraRightX = vect("walk:extraRightX");
  if (vect.exist("walk:extraRightY"))
    params.extraRightY = vect("walk:extraRightY");
  if (vect.exist("walk:extraRightZ"))
    params.extraRightZ = vect("walk:extraRightZ");
  if (vect.exist("walk:extraLeftYaw"))
    params.extraLeftYaw = vect("walk:extraLeftYaw");
  if (vect.exist("walk:extraLeftPitch"))
    params.extraLeftPitch = vect("walk:extraLeftPitch");
  if (vect.exist("walk:extraLeftRoll"))
    params.extraLeftRoll = vect("walk:extraLeftRoll");
  if (vect.exist("walk:extraRightYaw"))
    params.extraRightYaw = vect("walk:extraRightYaw");
  if (vect.exist("walk:extraRightPitch"))
    params.extraRightPitch = vect("walk:extraRightPitch");
  if (vect.exist("walk:extraRightRoll"))
    params.extraRightRoll = vect("walk:extraRightRoll");

  return params;
}

void IKWalk::boundPhase(double& phase)
{
  while (phase >= 1.0)
  {
    phase -= 1.0;
    // Bound to zero in case
    // of floating point error
    if (phase < 0.0)
    {
      phase = 0.0;
    }
  }
}

}  // namespace Leph
