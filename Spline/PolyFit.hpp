#ifndef LEPH_POLYFIT_HPP
#define LEPH_POLYFIT_HPP

#include <Eigen/Dense>
#include "Spline/Polynom.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"

namespace Leph
{
/**
 * PolyFit
 *
 * Fit one dimentionnal data
 * points by a polynom of given degree
 * and minimizing least square error
 */
class PolyFit
{
public:
  /**
   * Initialization with polynom
   * fitting degree
   */
  PolyFit(unsigned int degree);

  /**
   * Add a data point at given
   * time abscisse
   */
  void add(double t, double val);

  /**
   * Compute and return the fitted
   * polynom in least square sense
   */
  Polynom fitting();

  /**
   * Access to internal Linear Regression
   * instance
   */
  const SimpleLinearRegression& regression() const;

private:
  /**
   * Asked fitting degree
   */
  unsigned int _degree;

  /**
   * Internal Linear Regression instance
   */
  SimpleLinearRegression _regression;
};

}  // namespace Leph

#endif
