/*****************************************************************************/
/*! \file    math_basics.h
 *  \author  Rhoban Project
 *  \date    2011-02
 *  \brief   Basics resources for the math library
 *****************************************************************************/
#ifndef MATH_BASICS_H
#define MATH_BASICS_H
#include <math.h>
#include <vector>

/****************************************************************************/
// Constants
/****************************************************************************/

#define PI 3.141592653589793238462

/****************************************************************************/
// General Tools
/****************************************************************************/

#define real_abs(x) (((x) >= 0.0) ? (x) : -(x))
#define the_min(a, b) (((a) < (b)) ? (a) : (b))
#define the_max(a, b) (((a) > (b)) ? (a) : (b))
#define sign(a) (((a) < 0) ? -1 : 1)
int closest_int(double x);

template <class T>
struct triple
{
  T first;
  T second;
  T third;
  triple() : first(T()), second(T()), third(T())
  {
  }
  triple(const T& x, const T& y, const T& z) : first(x), second(y), third(z)
  {
  }
  template <class U>
  triple(const triple<U>& p) : first(p.first), second(p.second), third(p.third)
  {
  }
};

/** |x-y| */
double dist(double x, double y);

/****************************************************************************/
// Angles
/****************************************************************************/

#define DEG2RAD(x) ((x)*PI / 180.0)
#define RAD2DEG(x) ((x)*180.0 / PI)

/* return the argument of x + iy in [0, 2*Pi [ */
double argument(double x, double y);

/* return angle between -PI and PI */
double normalise_angle(double angle);

/* return angle between -180 and 180 */
double normalise_degree_angle(double angle);

/* return the shortest difference angle (rad) */
double angle_delta(double a1, double a2);

/* return the shortest difference angle (degree) */
double degree_angle_delta(double a1, double a2);

/* the polynom is defined by the list a_O a_1 ... a_n of
   its coefficients */
double eval_polynom(std::vector<double> P, double x);
std::vector<double> diff_polynom(std::vector<double> P);

/* the linear interpolation in x for [x1->y1, x2->y2] */
double linear_interpol(double x1, double y1, double x2, double y2, double x);

/* the base sigmoid from [0,1] to [0,1]
 * p determine the curvature of the sigmoid,
 * the derivative of the sigmoid is set to p
 * at 0.5. p \in ]0, + \infty [ */
double sigmoid(double x, double p);

bool isNaN(double x);

#endif /* MATH_BASICS_H */
/****************************************************************************/
/****************************************************************************/
