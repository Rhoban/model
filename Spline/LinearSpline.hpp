#ifndef LEPH_LINEARSPLINE_HPP
#define LEPH_LINEARSPLINE_HPP

#include "Spline/Spline.hpp"

namespace Leph
{
/**
 * LinearSpline
 *
 * Implementation of 3th order
 * polynomial splines
 */
class LinearSpline : public Spline
{
public:
  /**
   * Add a new point with its time and position value,
   */
  void addPoint(double time, double position);

private:
  /**
   * Simple point struture
   */
  struct Point
  {
    double time;
    double position;
  };

  /**
   * Points container
   */
  std::vector<Point> _points;

  /**
   * Recompute splines interpolation model
   */
  void computeSplines();
};

}  // namespace Leph

#endif
