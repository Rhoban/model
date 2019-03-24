#include <iostream>  //TODO
#include <stdexcept>
#include "Gradient/FiniteDifferenceGradient.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"

namespace Leph
{
void FiniteDifferenceGradient::addExperiment(const Vector& deltaParam, double fitness)
{
  if (_deltaParams.size() > 0 && (size_t)deltaParam.size() != dimension())
  {
    throw std::logic_error("FiniteDifferenceGradient invalid dimension");
  }

  _fitnesses.push_back(fitness);
  _deltaParams.push_back(deltaParam);

  update();
}

size_t FiniteDifferenceGradient::size() const
{
  return _fitnesses.size();
}

size_t FiniteDifferenceGradient::dimension() const
{
  if (_deltaParams.size() > 0)
  {
    return _deltaParams.front().size();
  }
  else
  {
    return 0;
  }
}

Vector FiniteDifferenceGradient::gradient() const
{
  return _gradient;
}

void FiniteDifferenceGradient::update()
{
  SimpleLinearRegression regression;

  for (size_t i = 0; i < size(); i++)
  {
    Vector input = _deltaParams[i];
    input.conservativeResize(dimension() + 1, Eigen::NoChange_t());
    input(dimension()) = 1.0;
    regression.add(input, _fitnesses[i]);
  }

  regression.regression();
  _gradient = regression.parameters().head(dimension());
}

}  // namespace Leph
