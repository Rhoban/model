#ifndef LEPH_CARTWALKGRADIENT_HPP
#define LEPH_CARTWALKGRADIENT_HPP

#include "Types/types.h"
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

namespace Leph
{
/**
 * CartWalkGradient
 *
 * Compute CartWalk outputs
 * differentiation with respect to
 * static parameters
 */
class CartWalkGradient
{
public:
  /**
   * Numerical differentiation of CartWalk
   * outputs with respect to parameters
   * at given phase, static and dynamic parameters.
   * Return a jacobian Matrix with outputs in rows
   * and parameters differentiation in cols.
   * Rows and Cols are indexed with respect to
   * walk VectorLabel indexes.
   * Used differentiation step could be given
   */
  Matrix differentiation(double phase, const VectorLabel& params, double diffStep = 0.0001);

private:
  /**
   * CartWalk Proxy instance
   */
  CartWalkProxy _walk;
};

}  // namespace Leph

#endif
