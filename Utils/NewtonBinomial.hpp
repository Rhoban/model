#ifndef LEPH_NEWTONBINOMIAL_HPP
#define LEPH_NEWTONBINOMIAL_HPP

#include "Utils/Combination.hpp"
#include "Spline/Polynom.hpp"

namespace Leph
{
/**
 * NewtonBinomial
 *
 * Implement Newton binomial
 * simple formulae and binding
 * with polynom structure
 */
class NewtonBinomial
{
public:
  /**
   * Expand the given formula (x + y)^degree
   * and return the polynom in x whose coefficient
   * are computed using binomial coefficient
   */
  static Polynom expandPolynom(double y, unsigned int degree);

private:
};

}  // namespace Leph

#endif
