#include <cmath>
#include "LinearRegression/SimpleLinearRegression.hpp"

namespace Leph
{
void SimpleLinearRegression::clear()
{
  _inputs = Matrix();
  _outputs = Vector();
  _params = Vector();
}

size_t SimpleLinearRegression::count() const
{
  return _inputs.cols();
}
size_t SimpleLinearRegression::dimension() const
{
  return _inputs.rows();
}

void SimpleLinearRegression::add(const Vector& input, double output, double weight)
{
  size_t size = count();
  size_t dim = dimension();
  double sqrtWeight = sqrt(weight);
  if (size == 0)
  {
    _inputs = sqrtWeight * input;
    _outputs = Vector(1);
    _outputs(0) = sqrtWeight * output;
  }
  else
  {
    if (dim != (size_t)input.size())
    {
      throw std::logic_error("SimpleLinearRegression invalid input size");
    }
    _outputs.conservativeResize(size + 1, Eigen::NoChange_t());
    _outputs(size) = sqrtWeight * output;
    _inputs.conservativeResize(Eigen::NoChange_t(), size + 1);
    _inputs.rightCols(1) = sqrtWeight * input;
  }
}

Vector SimpleLinearRegression::regression()
{
  if (count() == 0 || dimension() == 0)
  {
    throw std::logic_error("SimpleLinearRegression empty data");
  }

  Matrix XtX = _inputs * _inputs.transpose();
  Matrix XtY = _inputs * _outputs;
  _params = (XtX).llt().solve(XtY);

  return _params;
}

Vector SimpleLinearRegression::parameters() const
{
  return _params;
}

double SimpleLinearRegression::prediction(const Vector& input) const
{
  return _params.dot(input);
}

double SimpleLinearRegression::meanSquaredError() const
{
  if (count() == 0 || dimension() == 0)
  {
    throw std::logic_error("SimpleLinearRegression empty data");
  }

  Vector residuals = _outputs - (_params.transpose() * _inputs).transpose();
  return residuals.squaredNorm() / count();
}
double SimpleLinearRegression::rootMeanSquaredError() const
{
  return sqrt(meanSquaredError());
}

double SimpleLinearRegression::maxError() const
{
  Vector residuals = _outputs - (_params.transpose() * _inputs).transpose();
  return residuals.cwiseAbs().maxCoeff();
}

double SimpleLinearRegression::correlationCoefficient() const
{
  if (count() == 0 || dimension() == 0)
  {
    throw std::logic_error("SimpleLinearRegression empty data");
  }

  double mean = _outputs.sum() / count();
  Vector meanVect = mean * Vector::Ones(count());

  Vector sumSquared = _outputs - meanVect;
  Vector residuals = _outputs - (_params.transpose() * _inputs).transpose();
  double ss = sumSquared.squaredNorm();

  if (fabs(ss) < 0.0001)
  {
    return 1.0;
  }
  else
  {
    return 1.0 - residuals.squaredNorm() / ss;
  }
}

double SimpleLinearRegression::variance() const
{
  if (count() == 0 || dimension() == 0)
  {
    throw std::logic_error("SimpleLinearRegression empty data");
  }

  if (count() > 1)
  {
    Vector residuals = _outputs - (_params.transpose() * _inputs).transpose();
    return residuals.squaredNorm() / (count() - 1.0);
  }
  else
  {
    return 0.0;
  }
}

double SimpleLinearRegression::predictionBound(const Vector& input, bool withVariance) const
{
  if (count() == 0 || dimension() == 0)
  {
    throw std::logic_error("SimpleLinearRegression empty data");
  }

  double var = variance();

  Vector meanInput = Vector::Zero(dimension());
  for (size_t i = 0; i < count(); i++)
  {
    meanInput += _inputs.col(i);
  }
  meanInput *= 1.0 / count();

  double sumX = 0.0;
  for (size_t i = 0; i < count(); i++)
  {
    sumX += (_inputs.col(i) - meanInput).squaredNorm();
  }

  if (fabs(sumX) < 0.0001)
  {
    // 1.7 is about 95% confidence bound for 20 learning data point (Students law)
    return 1.7 * sqrt(var * ((withVariance ? 1.0 : 0.0) + 1.0 / count()));
  }
  else
  {
    // 1.7 is about 95% confidence bound for 20 learning data point (Students law)
    return 1.7 * sqrt(var * ((withVariance ? 1.0 : 0.0) + 1.0 / count() + (input - meanInput).squaredNorm() / sumX));
  }
}

void SimpleLinearRegression::print(std::ostream& os) const
{
  os << "SimpleLinearRegression count=" << count() << " dim=" << dimension() << std::endl;
  os << "Parameters: " << _params.transpose() << std::endl;
  os << "MSE: " << meanSquaredError() << std::endl;
  os << "RMSE: " << rootMeanSquaredError() << std::endl;
  os << "MaxE: " << maxError() << std::endl;
  os << "R^2: " << correlationCoefficient() << std::endl;
  os << "Var: " << variance() << std::endl;
  os << "Std: " << sqrt(variance()) << std::endl;
}

}  // namespace Leph
