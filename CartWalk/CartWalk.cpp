#include <iostream>
#include <cstdlib>
#include <math.h>
#include "CartWalk.h"

// Conversions
#ifndef DEG2RAD
#define DEG2RAD(x) (((x) / 180.0) * M_PI)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) (((x) / M_PI) * 180.0)
#endif

#define CARTABS(x) (((x) > 0) ? (x) : (-(x)))
#define CARTMIN(a, b) ((a) < (b) ? (a) : (b))
#define CARTMAX(a, b) ((a) > (b) ? (a) : (b))
#define CARTSMOOTHING 0.97
#define CARTSMOOTH(oldV, newV) (((oldV)*CARTSMOOTHING) + ((newV) * (1 - CARTSMOOTHING)))

namespace Rhoban
{
/**
 * Vous pouvez écrire du code qui sera exécuté à
 * l'initialisation ici
 */
CartWalk::CartWalk()
  : t(0.0)
  , timeGain(2.0)
  , xOffset(0.0)
  , yOffset(0.0)
  , zOffset(3.0)
  , yLat(0.0)
  , hipOffset(0.0)
  , riseGain(0.0)
  , swingGain(0.0)
  , swingHeight(0.0)
  , swingPhase(0.0)
  , armsGain(0)
  , stepGain(0.0)
  , lateralStepGain(0.0)
  , turn(0.0)
{
  swingForce = 0.0;
  riseRatio = 0.5;
  riseStepPhase = 0.0;
  isEnabled = false;
}

void CartWalk::setLength(double L1, double L2)
{
  legModel.femurLength = L1;
  legModel.tibiaLength = L2;
}

static double fMod(double t)
{
  while (t > 1.0)
    t -= 1.0;
  while (t < 0.0)
    t += 1.0;

  return t;
}

void CartWalk::tick(double elapsed)
{
  t = fMod(t + elapsed * timeGain);

  if (turn != turn || stepGain != stepGain || lateralStepGain != lateralStepGain)
  {
    printf("Nan as input!!!!!!!!!\n");
    return;
  }

  sLX = sLY = sLZ = sRX = sRY = sRZ = 0.0;

  double smoothing = 0.2;

  float rotationSpeed = turn / timeGain;
  float speed = stepGain / timeGain;
  float lateralSpeed = lateralStepGain / timeGain;

  // Speeds to flush
  float l_rotationSpeed, l_speed, l_lateralSpeed;
  float r_rotationSpeed, r_speed, r_lateralSpeed;
  float l_stepGain, r_stepGain;

  // XXX REMOVING SMOOTHING FOR STATELESS
  /*
  if (isEnabled) {
      l_rotationSpeed = CARTSMOOTH(l_rotationSpeed, rotationSpeed);
      l_speed = CARTSMOOTH(l_speed, speed);
      l_lateralSpeed = CARTSMOOTH(l_lateralSpeed, lateralSpeed);
      r_rotationSpeed = CARTSMOOTH(r_rotationSpeed, rotationSpeed);
      r_speed = CARTSMOOTH(r_speed, speed);
      r_lateralSpeed = CARTSMOOTH(r_lateralSpeed, lateralSpeed);
  } else {
      l_speed = l_lateralSpeed = l_rotationSpeed = 0;
      r_speed = r_lateralSpeed = r_rotationSpeed = 0;
  }
  */
  l_rotationSpeed = rotationSpeed;
  l_speed = speed;
  l_lateralSpeed = lateralSpeed;
  r_rotationSpeed = rotationSpeed;
  r_speed = speed;
  r_lateralSpeed = lateralSpeed;

  // Swinging
  swing.clear();
  swing.addPoint(0, -1, 0);
  swing.addPoint(swingForce / 2.0, -1, 0);
  swing.addPoint(0.5 - swingForce / 2.0, 1, 0);
  swing.addPoint(0.5 + swingForce / 2.0, 1, 0);
  swing.addPoint(1.0 - swingForce / 2.0, -1, 0);
  swing.addPoint(1, -1, 0);

  // Rising
  rise.clear();
  rise.addPoint(0, 0, 0);
  rise.addPoint(riseRatio / 2.0, 1, 0);
  rise.addPoint(riseRatio, 0, 0);
  rise.addPoint(1, 0, 0);

  // Stepping
  step.clear();
  step.addPoint(0, -0.5, 0);
  step.addPoint(riseRatio * 3 / 4.0, 0.5, 0);
  step.addPoint(1.0, -0.5, 0);

  // Computing rotationSpeed differential
  l_stepGain = l_speed;
  r_stepGain = r_speed;

  if (CARTABS(rotationSpeed) > 0)
  {
    double ratio = CARTMAX(0.0, speed - 8.0 * tan(DEG2RAD(CARTMIN(45, CARTABS(rotationSpeed)))));

    if (rotationSpeed < 0)
    {
      l_stepGain = ratio;
    }
    else
    {
      r_stepGain = ratio;
    }
  }
  double lX, lY, lZ, rX, rY, rZ;
  double swingValue = swing.getMod(t + swingPhase);

  // Walk enabling XXX REMOVING SMOOTHING FOR STATELESS
  /*
  if (isEnabled) {
      enabledRatio += 0.02;
  } else {
      enabledRatio -= 0.02;
  }
  if (enabledRatio > 1.0) {
      enabledRatio = 1.0;
  }
  if (enabledRatio < 0.0) {
      enabledRatio = 0.0;
  }
  */
  double enabledRatio = isEnabled;

  // Computing X, Y & Z
  lX = xOffset + enabledRatio * step.getMod(t) * l_stepGain;
  lY = yOffset + yLat + enabledRatio * (swingGain * swingValue + l_lateralSpeed * step.getMod(t));
  lZ = zOffset + enabledRatio * (rise.getMod(t) * riseGain + swingHeight * CARTABS(swingValue));

  rX = xOffset + enabledRatio * step.getMod(t + 0.5) * r_stepGain;
  rY = yOffset - yLat - enabledRatio * (swingGain * swingValue + r_lateralSpeed * step.getMod(t + 0.5));
  rZ = zOffset + enabledRatio * (rise.getMod(t + 0.5) * riseGain + swingHeight * CARTABS(swingValue));

  sLX = lX * (1 - smoothing) + smoothing * sLX;
  sLY = lY * (1 - smoothing) + smoothing * sLY;
  sLZ = lZ * (1 - smoothing) + smoothing * sLZ;
  sRX = rX * (1 - smoothing) + smoothing * sRX;
  sRY = rY * (1 - smoothing) + smoothing * sRY;
  sRZ = rZ * (1 - smoothing) + smoothing * sRZ;

  // printf("%g %g %g\n", lX, lY, lZ);

  // Computing IK
  // LegAngles left = legModel.compute(lX, lY, lZ);
  // LegAngles right = legModel.compute(rX, rY, rZ);
  LegAngles left = legModel.compute(sLX, sLY, sLZ);
  LegAngles right = legModel.compute(sRX, sRY, sRZ);

  // Setting angles
  a_l_hip_pitch = -RAD2DEG(left.hipLong) - hipOffset;
  a_l_hip_roll = -RAD2DEG(left.hipLat);
  a_l_hip_yaw = enabledRatio * step.getMod(t + riseStepPhase) * l_rotationSpeed;
  a_l_knee = RAD2DEG(left.knee);
  a_l_foot_pitch = RAD2DEG(left.ankleLong);
  a_l_foot_roll = RAD2DEG(left.ankleLat);

  a_r_hip_pitch = RAD2DEG(right.hipLong) + hipOffset;
  a_r_hip_roll = RAD2DEG(right.hipLat);
  a_r_hip_yaw = enabledRatio * step.getMod(t + 0.5 + riseStepPhase) * r_rotationSpeed;
  a_r_knee = -RAD2DEG(right.knee);
  a_r_foot_pitch = -RAD2DEG(right.ankleLong);
  a_r_foot_roll = -RAD2DEG(right.ankleLat);

  a_l_arm = -enabledRatio * step.getMod(t) * armsGain;
  a_r_arm = enabledRatio * step.getMod(t + 0.5) * armsGain;
}
}  // namespace Rhoban
