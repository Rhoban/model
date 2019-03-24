#ifndef LEPH_MEDIANLINEARREGRESSION_HPP
#define LEPH_MEDIANLINEARREGRESSION_HPP

#include <vector>
#include "Types/types.h"

namespace Leph
{
/**
 * MedianLinearRegression
 *
 * Implement a multi dimentional
 * linear regression using the
 * median parameter estimation
 * (Theil–Sen estimator)
 */
class MedianLinearRegression
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
  size_t size() const;
  size_t dimension() const;

  /**
   * Add given data point input vector
   * and output scalar
   */
  void add(const Vector& input, double output);

  /**
   * TODO
   */
  Vector regression();

private:
  /**
   * Registered points container
   * inputs and outputs vectors and values
   */
  std::vector<Vector> _inputs;
  std::vector<double> _outputs;
};

}  // namespace Leph

#endif
