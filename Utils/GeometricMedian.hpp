#ifndef LEPH_GEOMETRICMEDIAN_HPP
#define LEPH_GEOMETRICMEDIAN_HPP

#include <vector>
#include "Types/types.h"

namespace Leph
{
/**
 * GeometricMedian
 *
 * Implementation of "Vardi and Zhang (2000)"
 * algorithm computing spatial or geometric
 * median. It is the point given a set of multi
 * dimentional points which minimized the sum
 * of distance with all others points
 */
class GeometricMedian
{
public:
  /**
   * Add given point to internal
   * container
   */
  void add(const Vector& point);

  /**
   * Return the number of registered
   * points
   */
  size_t size() const;

  /**
   * Return the points dimension
   */
  size_t dimension() const;

  /**
   * Compute iteratively the geometric
   * median of given set points.
   * Convergence criterion is either
   * the loop number of iteration or
   * median update distance threshold.
   * Both threshold could be negative
   * to not be used.
   */
  Vector median(double deltaNorm, int maxLoopNumber) const;

private:
  /**
   * Points container
   */
  std::vector<Vector> _points;

  /**
   * Return the gravity center of
   * registered points
   */
  Vector mean() const;
};

}  // namespace Leph

#endif
