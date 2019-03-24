#ifndef LEPH_SIMPLELINEARREGRESSION_HPP
#define LEPH_SIMPLELINEARREGRESSION_HPP

#include <vector>
#include <iostream>
#include "Types/types.h"

namespace Leph
{
/**
 * SimpleLinearRegression
 *
 * Simple multidimentionnal
 * optionnaly weighted classic
 * linear regression
 * Least square fitted
 * No bayesian
 */
class SimpleLinearRegression
{
public:
  /**
   * Clear all registered data
   */
  void clear();

  /**
   * Return the number of registered data points
   * and input vector dimension
   */
  size_t count() const;
  size_t dimension() const;

  /**
   * Add given data point input vector
   * and output scalar
   */
  void add(const Vector& input, double output, double weight = 1.0);

  /**
   * Run the linear regression and return
   * the computed parameter vector
   */
  Vector regression();

  /**
   * Return fitted linear parameters
   */
  Vector parameters() const;

  /**
   * Compute and return the output given
   * the input vector and fitted parameters
   */
  double prediction(const Vector& input) const;

  /**
   * Return the mean (and root of) of squared residuals
   * (learning points versus fitted model prediction)
   */
  double meanSquaredError() const;
  double rootMeanSquaredError() const;

  /**
   * Return the maximum residual
   * (maximum error between leaning points
   * and fitted model prediction)
   */
  double maxError() const;

  /**
   * Return the coefficient of correlation between
   * 0 and 1 (quality of linear fitting)
   */
  double correlationCoefficient() const;

  /**
   * Return the estimated output variance
   */
  double variance() const;

  /**
   * Return the prediction error interval
   * (prediction +/- the bound) given the input point
   * (within about 95% confidence)
   * If withVariance is true, the bounds take into account
   * the estimated variance of output
   * (bound with real sampled noised data)
   * Else, the learning point noise is not take into account
   * (bound with theoric unnoised linear function data)
   */
  double predictionBound(const Vector& input, bool withVariance = true) const;

  /**
   * Print regression informations
   * (only available after the
   * regression is done)
   */
  void print(std::ostream& os = std::cout) const;

private:
  /**
   * Inputs Matrix
   */
  Matrix _inputs;

  /**
   * Outputs Vector
   */
  Vector _outputs;

  /**
   * Computed fitting parameters
   */
  Vector _params;
};

}  // namespace Leph

#endif
