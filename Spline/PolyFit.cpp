#include "Spline/PolyFit.hpp"

namespace Leph
{
PolyFit::PolyFit(unsigned int degree) : _degree(degree), _regression()
{
}

void PolyFit::add(double t, double val)
{
  Eigen::VectorXd inputs(_degree + 1);
  double expT = 1.0;
  for (size_t k = 0; k < _degree + 1; k++)
  {
    inputs(k) = expT;
    expT *= t;
  }
  _regression.add(inputs, val);
}

Polynom PolyFit::fitting()
{
  _regression.regression();

  Polynom polynom;
  polynom.getCoefs().resize(_degree + 1);
  for (size_t k = 0; k < _degree + 1; k++)
  {
    polynom.getCoefs()[k] = _regression.parameters()(k);
  }

  return polynom;
}

const SimpleLinearRegression& PolyFit::regression() const
{
  return _regression;
}

}  // namespace Leph
