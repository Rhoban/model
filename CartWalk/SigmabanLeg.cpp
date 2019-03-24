#include "CartWalk/linear_algebra.h"
#include "CartWalk/SigmabanLeg.h"

using namespace std;

LegAngles::LegAngles() : hipLat(0.0), hipLong(0.0), knee(0.0), ankleLong(0.0), ankleLat(0.0)
{
}

LegAngles::LegAngles(Matrix theta)
  : hipLat(theta[0]), hipLong(theta[1]), knee(theta[2]), ankleLong(theta[3]), ankleLat(theta[4])
{
}

Matrix LegAngles::toMatrix()
{
  return Matrix::mk_vector(5, hipLat, hipLong, knee, ankleLong, ankleLat);
}

Matrix SigmabanLeg::compute_direct_position(LegAngles thetas)
{
  double leg_length = sqrt(tibiaLength * tibiaLength + femurLength * femurLength -
                           2 * femurLength * tibiaLength * cos(M_PI - thetas.knee));
  double leg_triangle_hip_angle = al_kashi(tibiaLength, femurLength, leg_length);
  double leg_sagittal_angle = normalise_angle(thetas.hipLong - leg_triangle_hip_angle);
  double leg_Y = leg_length * sin(thetas.hipLat);
  double leg_sagittal_length = leg_length * cos(thetas.hipLat);
  double leg_X = leg_sagittal_length * sin(leg_sagittal_angle);
  double leg_Z = leg_sagittal_length * cos(leg_sagittal_angle);
  return Matrix::mk_vector(3, leg_X, leg_Y, tibiaLength + femurLength - leg_Z);
}

LegAngles SigmabanLeg::compute(double x, double y, double z)
{
  return compute(Matrix::mk_vector(3, x, y, z));
}

LegAngles SigmabanLeg::compute(Matrix feet_pos)
{
  LegAngles result;

  Matrix feet_pos_hip = feet_pos + Matrix::mk_vector(3, 0.0, 0.0, -(femurLength + tibiaLength));
  double X = feet_pos_hip[0];
  double Y = feet_pos_hip[1];
  double Z = feet_pos_hip[2];

  double d_hip_ankle = feet_pos_hip.norm2();
  if (d_hip_ankle > tibiaLength + femurLength)
  {
    result.knee = 0.0;
    result.hipLong = normalise_angle(argument(-Z, X));
    result.ankleLong = normalise_angle(-result.hipLong);
    result.hipLat = normalise_angle(argument(-Z, Y));
    result.ankleLat = normalise_angle(-result.hipLat);
    return result;
  }

  result.knee = normalise_angle(PI - al_kashi(d_hip_ankle, femurLength, tibiaLength));
  result.hipLat = normalise_angle(argument(-Z, Y));
  result.ankleLat = normalise_angle(-result.hipLat);

  double leg_triangle_hip_angle = al_kashi(tibiaLength, femurLength, d_hip_ankle);
  double feet_hip_angle = argument(-Z, X);
  result.hipLong = normalise_angle(leg_triangle_hip_angle + feet_hip_angle);

  result.ankleLong = normalise_angle(al_kashi(femurLength, tibiaLength, d_hip_ankle) - feet_hip_angle);

  return result;
}
