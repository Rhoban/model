#ifndef LEPH_POLYSPLINE_HPP
#define LEPH_POLYSPLINE_HPP

#include <vector>
#include <algorithm>

namespace Leph
{
/**
 * PolySpline
 *
 * Splines primitive based on third order polynoms
 * for smooth and differentiable function
 */
class PolySpline
{
public:
  /**
   * Internal point structure
   * and container
   */
  struct Point
  {
    double position;
    double value;
    double delta;
  };
  typedef std::vector<Point> Points;

  /**
   * Empty all added points and splines
   */
  void clear()
  {
    _points.clear();
    _splines.clear();
  }

  /**
   * Add a point with its x position, y value and
   * its derivative slope
   */
  void addPoint(double pos, double val, double delta)
  {
    struct Point point = { pos, val, delta };
    _points.push_back(point);
    computeSplines();
  }

  /**
   * Return the spline interpolation
   * for given x position
   */
  double get(double x) const
  {
    if (_points.size() == 0)
    {
      return 0.0;
    }
    else if (_points.size() == 1)
    {
      return _points.front().value;
    }
    else
    {
      for (size_t i = 0; i < _splines.size(); i++)
      {
        if (x >= _splines[i].min && x <= _splines[i].max)
        {
          return polynomValue((x - _splines[i].min) / (_splines[i].max - _splines[i].min), _splines[i].poly);
        }
      }
      return 0.0;
    }
  }

  /**
   * Return the spline interpolation value
   * with x bound between 0 and 1
   */
  double getMod(double x) const
  {
    if (x < 0.0)
    {
      x = 1.0 + (x - ((int)x / 1));
    }
    else if (x > 1.0)
    {
      x = (x - ((int)x / 1));
    }

    return get(x);
  }

  /**
   * Access to internal Points container
   */
  const Points& points() const
  {
    return _points;
  }

private:
  /**
   * Internal spline interval and polynom
   * structure and container
   */
  struct Polynom
  {
    double a;
    double b;
    double c;
    double d;
  };
  struct Spline
  {
    Polynom poly;
    double min;
    double max;
  };
  typedef std::vector<Spline> Splines;

  /**
   * Spline Points container
   */
  Points _points;

  /**
   * Splines container
   */
  Splines _splines;

  /**
   * Fast exponentation to compute
   * given polynom value
   */
  static double polynomValue(double t, const Polynom& p)
  {
    return p.d + t * (t * (p.a * t + p.b) + p.c);
  }

  /**
   * Fit a polynom between 0 and 1 with
   * given value and slope
   */
  static Polynom polynomFit(double val1, double delta1, double val2, double delta2)
  {
    struct Polynom polynom = { 2.0 * val1 + delta1 + delta2 - 2.0 * val2,
                               3.0 * val2 - 2.0 * delta1 - 3.0 * val1 - delta2, delta1, val1 };

    return polynom;
  }

  /**
   * Recompute splines interpolation model
   */
  void computeSplines()
  {
    _splines.clear();
    if (_points.size() < 2)
    {
      return;
    }

    std::sort(_points.begin(), _points.end(),
              [](const Point& p1, const Point& p2) -> bool { return p1.position < p2.position; });

    for (size_t i = 1; i < _points.size(); i++)
    {
      if (fabs(_points[i - 1].position - _points[i].position) < 0.00001)
      {
        continue;
      }
      struct Spline spline = { polynomFit(_points[i - 1].value, _points[i - 1].delta, _points[i].value,
                                          _points[i].delta),
                               _points[i - 1].position, _points[i].position };

      _splines.push_back(spline);
    }
  }
};

}  // namespace Leph

#endif
