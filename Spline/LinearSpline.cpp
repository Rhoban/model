#include <stdexcept>
#include <algorithm>
#include "Spline/LinearSpline.hpp"

namespace Leph
{
void LinearSpline::addPoint(double time, double position)
{
  _points.push_back({ time, position });
  computeSplines();
}

void LinearSpline::computeSplines()
{
  Spline::_splines.clear();
  if (_points.size() < 2)
  {
    return;
  }

  std::sort(_points.begin(), _points.end(), [](const Point& p1, const Point& p2) -> bool { return p1.time < p2.time; });

  for (size_t i = 1; i < _points.size(); i++)
  {
    double time = _points[i].time - _points[i - 1].time;
    if (time > 0.00001)
    {
      Polynom poly(1);
      poly(0) = _points[i - 1].position;
      poly(1) = (_points[i].position - _points[i - 1].position) / time;
      Spline::_splines.push_back({ poly, _points[i - 1].time, _points[i].time });
    }
  }
}

}  // namespace Leph
