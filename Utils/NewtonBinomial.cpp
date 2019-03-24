#include <cmath>
#include "Utils/NewtonBinomial.hpp"

#include <iostream>

namespace Leph
{
Polynom NewtonBinomial::expandPolynom(double y, unsigned int degree)
{
  Combination combination;

  Polynom polynom;
  polynom.getCoefs().resize(degree + 1);

  for (size_t k = 0; k <= degree; k++)
  {
    polynom.getCoefs()[k] = combination.binomialCoefficient(k, degree) * pow(y, degree - k);
  }

  return polynom;
}

}  // namespace Leph
