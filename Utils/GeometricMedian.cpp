#include <stdexcept>
#include "Utils/GeometricMedian.hpp"

namespace Leph
{
void GeometricMedian::add(const Vector& point)
{
  if (size() > 1 && (size_t)point.size() != dimension())
  {
    throw std::logic_error("GeometricMedian invalid dimension");
  }

  _points.push_back(point);
}

size_t GeometricMedian::size() const
{
  return _points.size();
}

size_t GeometricMedian::dimension() const
{
  if (size() > 1)
  {
    return _points.front().size();
  }
  else
  {
    return 0;
  }
}

Vector GeometricMedian::median(double deltaNorm, int maxLoopNumber) const
{
  Vector median = mean();

  for (int k = 0; maxLoopNumber < 0 || k < maxLoopNumber; k++)
  {
    double eta = 0.0;
    double sumInvDist = 0.0;
    Vector sumFracX = Vector::Zero(dimension());
    Vector sumFracXSubMedian = Vector::Zero(dimension());
    for (size_t i = 0; i < size(); i++)
    {
      double dist = (median - _points[i]).norm();
      if (dist < 0.001)
      {
        eta = 1.0;
      }
      else
      {
        sumInvDist += 1.0 / dist;
        sumFracX += (1.0 / dist) * _points[i];
        sumFracXSubMedian += (1.0 / dist) * (_points[i] - median);
      }
    }
    Vector T = (1.0 / sumInvDist) * sumFracX;
    Vector R = sumInvDist * sumFracXSubMedian;
    double normR = R.norm();
    double gamma = (eta / normR > 1.0 ? 1.0 : eta / normR);
    Vector tmpMedian = (1.0 - gamma) * T + gamma * median;
    double deltaMedian = (tmpMedian - median).norm();
    if (deltaNorm > 0.0 && deltaMedian < deltaNorm)
    {
      break;
    }
    median = tmpMedian;
  }

  return median;
}

Vector GeometricMedian::mean() const
{
  Vector sum = Vector::Zero(dimension());

  for (size_t i = 0; i < size(); i++)
  {
    sum += _points[i];
  }

  if (size() > 0)
  {
    sum = (1.0 / size()) * sum;
  }

  return sum;
}

}  // namespace Leph
