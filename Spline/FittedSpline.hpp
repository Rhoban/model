#ifndef LEPH_FITTEDSPLINE_HPP
#define LEPH_FITTEDSPLINE_HPP

#include <vector>
#include "Spline/Spline.hpp"

namespace Leph
{
/**
 * FittedSpline
 *
 * Fit polynomial spline on given
 * unidimentional data points.
 * Two different fitting strategy are
 * implemented.
 */
class FittedSpline : public Spline
{
public:
  /**
   * Add a point for fitting
   */
  void addPoint(double x, double y);

  /**
   * Compute fitting spline on registered
   * data points. Spline knots are placed on
   * data points extremum (works only on noiseless data).
   * Each part is fitted independently by a polynom whose
   * degree is chosen to minimize fitting point error.
   * No continuity bound are ensure.
   * maxError is maximum error allowed between given
   * data point and fitted model prediction.
   * if throwError is true, runtime_error is thrown if
   * no good fit is found.
   * Return true if fit is successful.
   */
  bool fittingPieces(double maxError, bool throwError);

  /**
   * Compute fitting polynoms on registered
   * data points. All fitted polynoms have given degree.
   * Position and derivatives continuity are ensure
   * at splines knots.
   * Knots are generated uniformally every
   * sequenceLength data point.
   * All splines are computed alltogether ensuring bounds
   * contraints using linear regression.
   */
  void fittingGlobal(unsigned int degree, unsigned int sequenceLength);

  /**
   * Fit given points with cubic/smooth polynoms
   * every sequenceLength of points.
   * Polynom velocity (or acceleration)
   * are computed numerically.
   * Noised data would not fit well.
   */
  void fittingCubic(unsigned int sequenceLength);
  void fittingSmooth(unsigned int sequenceLength);

  /**
   * Fit given points with pieces of Polynom of given
   * degree. No continuity at knots are ensure. Knots are
   * chosen in preference at extremum point with optional
   * time minimum and maximum bounds.
   * Data must not be noised and must have clear extremum.
   */
  double fittingPolynomPieces(unsigned int degree, double minTimeLength = -1.0, double maxTimeLength = -1.0);

  /**
   * Fit a smooth 5 degress splines on data points using
   * CMA-ES global optimisation.
   * The number of spline parts, min and max time bounds
   * and the number of maximum CMA-ES iterations and restart is given.
   * If isCycle is true, the fitted spline is supposed periodic.
   * Best founded RMSE is returned.
   */
  double fittingSmoothCMAES(unsigned int number, bool isCycle, double minTime, double maxTime,
                            unsigned int maxIterations, unsigned int restarts);

private:
  /**
   * Point typedef
   */
  typedef std::pair<double, double> Point;

  /**
   * Added points container
   */
  std::vector<Point> _points;

  /**
   * Check data size and sort
   * registered points by time
   */
  void prepareData();
};

}  // namespace Leph

#endif
