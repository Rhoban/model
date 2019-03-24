/*****************************************************************************/
/*! \file    math_basics.cpp
 *  \author  Rhoban Project
 *  \date    2011-02
 *  \brief   Basics resources for the math library
 *****************************************************************************/
#include <iostream>
#include <stdlib.h>
#include <vector>
#include "math_basics.h"
using namespace std;

/*****************************************************************************/

double argument(double x, double y)
{
  double res;
  if (x == 0)
    if (y < 0)
      res = 3 * PI / 2.0;
    else
      res = PI / 2.0;
  else
  {
    double a = atan(y / x);
    if (x >= 0)
      res = a;
    else
      res = a + PI;
  }

  while (res < 0.0)
    res += 2 * PI;
  while (res >= 2 * PI)
    res -= 2 * PI;
  return res;
}

/*****************************************************************************/

double normalise_angle(double angle)
{
  while (angle < -PI)
    angle += 2 * PI;
  while (angle >= PI)
    angle -= 2 * PI;
  return angle;
}

double normalise_degree_angle(double angle)
{
  while (angle < -180.0)
    angle += 360;
  while (angle >= 180.0)
    angle -= 360;
  return angle;
}

double angle_delta(double a1, double a2)
{
  double delta = a2 - a1;
  return normalise_angle(delta);
}

double degree_angle_delta(double a1, double a2)
{
  double delta = a2 - a1;
  return normalise_degree_angle(delta);
}

/*****************************************************************************/

double dist(double x, double y)
{
  if (x > y)
    return x - y;
  else
    return y - x;
}

double eval_polynom(vector<double> P, double x)
{
  double y = P[P.size() - 1];
  for (int i = (int)P.size() - 2; i >= 0; i--)
  {
    y = P[i] + x * y;
  }
  return y;
}

vector<double> diff_polynom(vector<double> P)
{
  vector<double> diffP;
  for (int i = 1; i < (int)P.size(); i++)
    diffP.push_back(i * P[i]);
  return diffP;
}

double linear_interpol(double x1, double y1, double x2, double y2, double x)
{
  if (x1 == x2)
    return (y1 + y2) / 2;
  return y1 + (x - x1) / (x2 - x1) * (y2 - y1);
}

double sigmoid(double x, double p)
{
  if (x <= 0)
    return 0.0;
  if (x >= 1.0)
    return 1.0;
  if (x <= 0.5)
    return 0.5 * pow(2 * x, p);
  return 1 - sigmoid(1 - x, p);
}

int closest_int(double x)
{
  int n = 0;
  int dn;
  if (x > 0)
    dn = 1;
  else
    dn = -1;
  while (real_abs(x - n) > 0.5)
    n += dn;
  return n;
}

bool isNaN(double x)
{
  return !(x == x);
}

/*****************************************************************************/
/*****************************************************************************/
