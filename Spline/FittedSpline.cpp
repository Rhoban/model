#include <algorithm>
#include <libcmaes/cmaes.h>
#include "Spline/FittedSpline.hpp"
#include "Spline/PolyFit.hpp"
#include "Spline/CubicSpline.hpp"
#include "Spline/SmoothSpline.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"
#include "Utils/NewtonBinomial.hpp"

namespace Leph
{
void FittedSpline::addPoint(double x, double y)
{
  _points.push_back({ x, y });
}

bool FittedSpline::fittingPieces(double maxError, bool throwError)
{
  // Data check
  if (_points.size() < 3)
  {
    throw std::logic_error("FittedSpline not enough points");
  }

  // Sort data
  prepareData();

  // Cut x axis into sequences
  //(select spline knots points)
  // by founding data extremum
  std::vector<std::pair<size_t, size_t>> parts;
  size_t beginIndex = 0;
  bool isIncreasing;
  // Find initial trend
  if (_points[1].second > _points[0].second + 1e-6)
  {
    isIncreasing = true;
  }
  else
  {
    isIncreasing = false;
  }
  for (size_t i = 1; i < _points.size(); i++)
  {
    if (isIncreasing && _points[i].second < _points[i - 1].second - 1e-6)
    {
      parts.push_back({ beginIndex, i - 1 });
      beginIndex = i - 1;
      isIncreasing = false;
    }
    else if (!isIncreasing && _points[i].second > _points[i - 1].second + 1e-6)
    {
      parts.push_back({ beginIndex, i - 1 });
      beginIndex = i - 1;
      isIncreasing = true;
    }
  }
  parts.push_back({ beginIndex, _points.size() - 1 });

  // Return value
  bool isFitSuccessful = true;

  // Compute linear regression for each parts
  // to find best polynomial fit
  for (size_t i = 0; i < parts.size(); i++)
  {
    // Try to fit with polynom of increasing degree
    unsigned int degree = 1;
    // Best fit found for unsuccessful fit
    double bestError = 0;
    unsigned int bestDegree = 0;
    Vector bestParams;
    while (true)
    {
      // Polynomial simple linear regression
      PolyFit fit(degree);
      for (size_t j = parts[i].first; j <= parts[i].second; j++)
      {
        fit.add(_points[j].first - _points[parts[i].first].first, _points[j].second);
      }
      Polynom polynom = fit.fitting();
      // Compute max fitting error
      double error = fit.regression().maxError();
      // Store best fit
      if (bestDegree == 0 || bestError > error)
      {
        bestDegree = degree;
        bestError = error;
        bestParams = fit.regression().parameters();
      }
      // Iterations are stopped if error threshold is meet
      // or degree is too high for data points available
      if (degree >= parts[i].second - parts[i].first || bestError <= maxError)
      {
        // If the fit is non successful, we can throw
        // exception or return false value
        if (bestError > maxError)
        {
          isFitSuccessful = false;
          if (throwError)
          {
            throw std::runtime_error("FittedSpline unable to fit data point");
          }
        }
        // Save computed fitting polynom to Splines container
        Spline::_splines.push_back({ polynom, _points[parts[i].first].first, _points[parts[i].second].first });
        // Go to next spline part
        break;
      }
      else
      {
        // Else, continue iteration with higher degree
        degree++;
      }
    }
  }

  return isFitSuccessful;
}

void FittedSpline::fittingGlobal(unsigned int degree, unsigned int sequenceLength)
{
  // Data check
  if (_points.size() == 0)
  {
    throw std::logic_error("FittedSpline not enough points");
  }

  // Sort data
  prepareData();

  // Choose spline knots uniformally
  std::vector<double> knots;
  for (size_t i = 1; i < _points.size() - sequenceLength / 2; i++)
  {
    if (i % sequenceLength == 0)
    {
      knots.push_back(_points[i].first);
    }
  }

  // Prepare smooth spline linear regression with position
  // and derivatives continuity
  // Data are: 1, x, x^2, ..., x^d, (x-knot1)^d, (x-knot2)^d, ...
  SimpleLinearRegression regression;
  for (size_t i = 0; i < _points.size(); i++)
  {
    Eigen::VectorXd inputs(degree + 1 + knots.size());
    double expT = 1.0;
    for (size_t k = 0; k < degree + 1; k++)
    {
      inputs(k) = expT;
      expT *= _points[i].first - _points.front().first;
    }
    for (size_t k = 0; k < knots.size(); k++)
    {
      if (_points[i].first < knots[k])
      {
        inputs(degree + 1 + k) = 0.0;
      }
      else
      {
        inputs(degree + 1 + k) = pow(_points[i].first - knots[k], degree);
      }
    }
    regression.add(inputs, _points[i].second);
  }

  // Run regression
  regression.regression();

  // Add first spline part
  Polynom polynomFirst(degree);
  for (size_t i = 0; i < degree + 1; i++)
  {
    polynomFirst(i) += regression.parameters()(i);
  }
  // Add the spline to the container
  double boundMinFirst = _points.front().first;
  double boundMaxFirst;
  if (knots.size() > 0)
  {
    boundMaxFirst = knots.front();
  }
  else
  {
    boundMaxFirst = _points.back().first;
  }
  Spline::_splines.push_back({ polynomFirst, boundMinFirst, boundMaxFirst });
  // Add all remaining spline parts beginning by on a knot
  for (size_t k = 0; k < knots.size(); k++)
  {
    Polynom polynom(degree);
    for (size_t i = 0; i < degree + 1; i++)
    {
      polynom(i) += regression.parameters()(i);
    }
    // Convertion from (x-knot)^d form to
    // coefficient values
    for (size_t i = 0; i <= k; i++)
    {
      Polynom tmp = NewtonBinomial::expandPolynom(_points.front().first - knots[i], degree);
      tmp *= regression.parameters()(degree + 1 + i);
      polynom += tmp;
    }
    // Add it to Spline container
    double boundMin = knots[k];
    double boundMax;
    if (k == knots.size() - 1)
    {
      boundMax = _points.back().first;
    }
    else
    {
      boundMax = knots[k + 1];
    }
    // Shift polynom on x axis for spline interface compliance
    polynom.shift(knots[k] - _points.front().first);
    Spline::_splines.push_back({ polynom, boundMin, boundMax });
  }
}

void FittedSpline::fittingCubic(unsigned int sequenceLength)
{
  // Data check
  if (_points.size() < 3)
  {
    throw std::logic_error("FittedSpline not enough points");
  }

  prepareData();

  // Fit cubic splines
  CubicSpline cubic;

  // Add first point
  double dYFirst = _points[1].second - _points[0].second;
  double dTFirst = _points[1].first - _points[0].first;
  double velFirst = dYFirst / dTFirst;
  cubic.addPoint(_points[0].first, _points[0].second, velFirst);

  // Add point every sequenceLength
  for (size_t i = 1; i < _points.size() - sequenceLength / 2; i++)
  {
    if (i % sequenceLength == 0)
    {
      double dY = _points[i + 1].second - _points[i - 1].second;
      double dT = _points[i + 1].first - _points[i - 1].first;
      if (dT <= 0.0)
      {
        throw std::logic_error("FittedSpline differentiation error");
      }
      double vel = dY / dT;
      cubic.addPoint(_points[i].first, _points[i].second, vel);
    }
  }

  // Add last point
  size_t size = _points.size();
  double dYEnd = _points[size - 1].second - _points[size - 2].second;
  double dTEnd = _points[size - 1].first - _points[size - 2].first;
  double velEnd = dYEnd / dTEnd;
  cubic.addPoint(_points[size - 1].first, _points[size - 1].second, velEnd);

  // Copy spline data
  Spline::operator=(cubic);
}

void FittedSpline::fittingSmooth(unsigned int sequenceLength)
{
  // Data check
  if (_points.size() < 5)
  {
    throw std::logic_error("FittedSpline not enough points");
  }

  prepareData();

  // Fit smooth splines
  SmoothSpline smooth;

  // Add first point
  double dYFirst = _points[1].second - _points[0].second;
  double dTFirst = _points[1].first - _points[0].first;
  double dTFirst2 = _points[2].first - _points[1].first;
  double velFirst = dYFirst / dTFirst;
  double accFirst = (_points[2].second - 2.0 * _points[1].second + _points[0].second) / (dTFirst * dTFirst2);
  smooth.addPoint(_points[0].first, _points[0].second, velFirst, accFirst);

  // Add point every sequenceLength
  for (size_t i = 2; i < _points.size() - sequenceLength / 2; i++)
  {
    if (i % sequenceLength == 0)
    {
      double dY = _points[i + 1].second - _points[i - 1].second;
      double dT = _points[i + 1].first - _points[i - 1].first;
      if (dT <= 0.0)
      {
        throw std::logic_error("FittedSpline differentiation error");
      }
      double vel = dY / dT;
      double acc = (_points[i + 1].second - 2.0 * _points[i].second + _points[i - 1].second) /
                   ((_points[i + 1].first - _points[i].first) * (_points[i].first - _points[i - 1].first));
      smooth.addPoint(_points[i].first, _points[i].second, vel, acc);
    }
  }

  // Add last point
  size_t size = _points.size();
  double dYEnd = _points[size - 1].second - _points[size - 2].second;
  double dTEnd = _points[size - 1].first - _points[size - 2].first;
  double dTEnd2 = _points[size - 2].first - _points[size - 3].first;
  double velEnd = dYEnd / dTEnd;
  double accEnd =
      (_points[size - 1].second - 2.0 * _points[size - 2].second + _points[size - 3].second) / (dTEnd * dTEnd2);
  smooth.addPoint(_points[size - 1].first, _points[size - 1].second, velEnd, accEnd);

  // Copy spline data
  Spline::operator=(smooth);
}

double FittedSpline::fittingPolynomPieces(unsigned int degree, double minTimeLength, double maxTimeLength)
{
  // Data check
  if (_points.size() < 3)
  {
    throw std::logic_error("FittedSpline not enough points");
  }

  // Sort data
  prepareData();

  // Cut x axis into sequences
  //(select spline knots points)
  // by founding data extremum and
  // taking into account time min/max length
  std::vector<std::pair<size_t, size_t>> parts;
  size_t beginIndex = 0;
  bool isIncreasing;
  // Find initial trend
  if (_points[1].second > _points[0].second + 1e-6)
  {
    isIncreasing = true;
  }
  else
  {
    isIncreasing = false;
  }
  for (size_t i = 1; i < _points.size(); i++)
  {
    // Choose when to create a new part
    bool doAdd = false;
    // Detect (noiseless) extremum
    if (isIncreasing && _points[i].second < _points[i - 1].second - 1e-6)
    {
      doAdd = true;
      isIncreasing = false;
    }
    else if (!isIncreasing && _points[i].second > _points[i - 1].second + 1e-6)
    {
      isIncreasing = true;
      doAdd = true;
    }
    // Check min/max time length
    double currentLength = _points[i].first - _points[beginIndex].first;
    if (maxTimeLength > 0.0 && currentLength >= maxTimeLength)
    {
      // Force adding to time length is too long
      doAdd = true;
    }
    if (doAdd && minTimeLength > 0.0 && currentLength < currentLength)
    {
      // Cancel adding if time length is too short
      doAdd = false;
    }
    // Add the new part
    if (doAdd)
    {
      parts.push_back({ beginIndex, i - 1 });
      beginIndex = i - 1;
    }
  }
  parts.push_back({ beginIndex, _points.size() - 1 });

  // Compute linear regression for each parts
  // to find best polynomial fit of given degree
  double maxError = 0.0;
  for (size_t i = 0; i < parts.size(); i++)
  {
    // Polynomial simple linear regression
    PolyFit fit(degree);
    for (size_t j = parts[i].first; j <= parts[i].second; j++)
    {
      fit.add(_points[j].first - _points[parts[i].first].first, _points[j].second);
    }
    Polynom polynom = fit.fitting();
    // Compute max fitting error
    double error = fit.regression().maxError();
    if (error > maxError)
    {
      maxError = error;
    }
    // Save computed fitting polynom to Splines container
    Spline::_splines.push_back({ polynom, _points[parts[i].first].first, _points[parts[i].second].first });
  }

  return maxError;
}

double FittedSpline::fittingSmoothCMAES(unsigned int number, bool isCycle, double minTime, double maxTime,
                                        unsigned int maxIterations, unsigned int restarts)
{
  // Data check
  if (_points.size() < 3)
  {
    throw std::logic_error("FittedSpline not enough points");
  }

  // Sort data
  prepareData();

  // Lambda check parameters time bounds
  auto checkParams = [](const Eigen::VectorXd& params) -> double {
    for (size_t i = 3; i < (size_t)params.size() - 3; i += 4)
    {
      if (params(i) <= 0.01)
      {
        return 1000.0 + 1000.0 * (0.01 - params(i));
      }
      if (params(i) >= 0.99)
      {
        return 1000.0 + 1000.0 * (params(i) - 0.99);
      }
      if (i >= 4 && params(i) <= params(i - 4) + 0.01)
      {
        return 1000.0 + 1000.0 * (params(i - 4) + 0.01 - params(i));
      }
    }
    return 0.0;
  };

  // Lambda build smooth spline from parameters
  auto buildSpline = [this, isCycle, minTime, maxTime](const Eigen::VectorXd& params) -> SmoothSpline {
    size_t size = params.size();
    // Get min/max bounds
    double lengthTime = maxTime - minTime;
    // Build the spline
    SmoothSpline spline;
    spline.addPoint(minTime, params(0), params(1), params(2));
    for (size_t i = 3; i < size - 3; i += 4)
    {
      spline.addPoint(params(i) * lengthTime + minTime, params(i + 1), params(i + 2), params(i + 3));
    }
    if (isCycle)
    {
      spline.addPoint(maxTime, params(0), params(1), params(2));
    }
    else
    {
      spline.addPoint(maxTime, params(size - 3), params(size - 2), params(size - 1));
    }

    return spline;
  };

  // Lambda score parameters RMSE
  libcmaes::FitFuncEigen fitness = [this, checkParams, buildSpline](const Eigen::VectorXd& params) -> double {
    double check = checkParams(params);
    if (check > 0.0)
    {
      return check;
    }
    SmoothSpline spline = buildSpline(params);

    double sumError = 0.0;
    unsigned long count = 0;
    for (size_t i = 0; i < this->_points.size(); i++)
    {
      double t = this->_points[i].first;
      double val = this->_points[i].second;
      double error = spline.pos(t) - val;
      sumError += error * error;
      count++;
    }

    return sqrt(sumError / count);
  };

  // Initial parameters
  size_t size = 3 + 4 * number;
  if (!isCycle)
  {
    size += 3;
  }
  Eigen::VectorXd initialParams(size);
  initialParams(0) = _points.front().second;
  initialParams(1) = 0.0;
  initialParams(2) = 0.0;
  for (size_t i = 0; i < number; i++)
  {
    size_t index = i * 4 + 3;
    double ratio = ((double)i + 1.0) / ((double)number + 1.0);
    initialParams(index + 0) = ratio;
    initialParams(index + 1) = (1.0 - ratio) * _points.front().second + ratio * _points.back().second;
    initialParams(index + 2) = 0.0;
    initialParams(index + 3) = 0.0;
  }
  if (!isCycle)
  {
    initialParams(size - 3) = _points.back().second;
    initialParams(size - 2) = 0.0;
    initialParams(size - 1) = 0.0;
  }

  // Run optimization
  libcmaes::CMAParameters<> cmaparams(initialParams, -1.0, 20);
  cmaparams.set_quiet(false);
  cmaparams.set_mt_feval(true);
  cmaparams.set_str_algo("abipop");
  cmaparams.set_elitism(true);
  cmaparams.set_restarts(restarts);
  cmaparams.set_max_iter(maxIterations);

  // Run optimization
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);

  // Retrieve best Spline and score
  Eigen::VectorXd bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
  double score = cmasols.get_best_seen_candidate().get_fvalue();
  SmoothSpline bestSpline = buildSpline(bestParams);

  // Copy spline data
  Spline::operator=(bestSpline);

  return score;
}

void FittedSpline::prepareData()
{
  // Clear spline
  Spline::_splines.clear();

  // Sort all points on x axis
  std::sort(_points.begin(), _points.end(),
            [](const Point& p1, const Point& p2) -> bool { return p1.first < p2.first; });
}

}  // namespace Leph
