#ifndef LEPH_FINITEDIFFERENCEGRADIENT_HPP
#define LEPH_FINITEDIFFERENCEGRADIENT_HPP

#include <vector>
#include "Types/types.h"

namespace Leph
{
/**
 * FiniteDifferenceGradient
 *
 * Implementation of iterative
 * gradient based on simple finite
 * difference and viewed as a regression
 * problem
 */
class FiniteDifferenceGradient
{
public:
  /**
   * Register a new couple delta parameters tried
   * and fitness reward received.
   * Update current gradient estimate.
   */
  void addExperiment(const Vector& deltaParam, double fitness);

  /**
   * Return the number of registered experiments
   */
  size_t size() const;

  /**
   * Return the dimention of registered
   * input parameters
   */
  size_t dimension() const;

  /**
   * Return the estimated gradient
   * Vector of delta parameters
   */
  Vector gradient() const;

private:
  /**
   * Measured fitness (outputs)
   * container
   */
  std::vector<double> _fitnesses;

  /**
   * Used delta parameters container
   */
  std::vector<Vector> _deltaParams;

  /**
   * Current found parameters
   * gradient vector by regression
   */
  Vector _gradient;

  /**
   * Recompute and update the gradient
   * estimation
   */
  void update();
};

}  // namespace Leph

#endif
