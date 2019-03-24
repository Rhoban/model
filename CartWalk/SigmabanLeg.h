#ifndef SIGMABAN_LEG_H
#define SIGMABAN_LEG_H
#include "CartWalk/linear_algebra.h"

class LegAngles
{
public:
  /* Angles are given in rad. Zero position is
   * when the leg is vertical and the foot horizontal. */
  double hipLat;
  double hipLong;
  double knee;
  double ankleLong;
  double ankleLat;

  LegAngles();
  LegAngles(Matrix thetas);
  Matrix toMatrix();
};

class SigmabanLeg
{
public:
  double femurLength;
  double tibiaLength;

  /* (x,y,z) is the position of the ankle in the frame
   * attached to the zero position of the ankle when the robot stand up.
   * x goes in the sagittal direction,
   * y goes to the left in the frontal direction, and z goes up in the vertical
   * direction. */

  LegAngles compute(double x, double y, double z);
  LegAngles compute(Matrix fee_pos);

  /* compute the position of the ankle in the cartesian (x,y,z) space
   * from the motor angles */
  Matrix compute_direct_position(LegAngles thetas);
};

#endif
