#include <cmath>
#include <stdexcept>
#include "Utils/GaussianDistribution.hpp"
#include "Utils/Angle.h"

namespace Leph
{
GaussianDistribution::GaussianDistribution()
  : _mean(), _covariance(), _isCircular(), _haveCircular(false), _choleskyDecomposition(), _determinant(0.0)
{
}

GaussianDistribution::GaussianDistribution(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance,
                                           const Eigen::VectorXi& isCircular)
  : _mean(mean)
  , _covariance(covariance)
  , _isCircular()
  , _haveCircular(false)
  , _choleskyDecomposition()
  , _determinant(0.0)
{
  // Check size
  if (mean.size() != covariance.rows() || mean.size() != covariance.cols())
  {
    throw std::logic_error("GaussianDistribution invalid input size");
  }
  if (isCircular.size() != 0 && isCircular.size() != mean.size())
  {
    throw std::logic_error("GaussianDistribution invalid circular size");
  }

  // Circular initialization
  if (isCircular.size() == mean.size())
  {
    _isCircular = isCircular;
    // Normalization
    for (size_t i = 0; i < (size_t)_isCircular.size(); i++)
    {
      if (_isCircular(i) != 0)
      {
        _isCircular(i) = 1;
        _mean(i) = AngleBound(_mean(i));
        _haveCircular = true;
      }
    }
  }
  else
  {
    _isCircular = Eigen::VectorXi::Zero(mean.size());
    _haveCircular = false;
  }

  // Cholesky decomposition of covariance matrix
  computeDecomposition();
}

size_t GaussianDistribution::dimension() const
{
  return _mean.size();
}

const Eigen::VectorXd& GaussianDistribution::mean() const
{
  return _mean;
}
const Eigen::MatrixXd& GaussianDistribution::covariance() const
{
  return _covariance;
}

const Eigen::VectorXi& GaussianDistribution::isCircular() const
{
  return _isCircular;
}

Eigen::VectorXd GaussianDistribution::sample(std::default_random_engine& engine, bool wrapAngles) const
{
  // Draw normal unit vector
  Eigen::VectorXd unitRand(_mean.size());
  std::normal_distribution<double> dist(0.0, 1.0);
  for (size_t i = 0; i < (size_t)unitRand.size(); i++)
  {
    unitRand(i) = dist(engine);
  }

  // Compute the random generated point
  Eigen::VectorXd point = _mean + _choleskyDecomposition * unitRand;
  // Angle normalization
  if (_haveCircular && !wrapAngles)
  {
    for (size_t i = 0; i < (size_t)_isCircular.size(); i++)
    {
      if (_isCircular(i) != 0)
      {
        point(i) = AngleBound(point(i));
      }
    }
  }

  return point;
}

double GaussianDistribution::probability(const Eigen::VectorXd& point, bool wrapAngles) const
{
  size_t size = _mean.size();
  // Compute distance from mean
  Eigen::VectorXd delta = computeDistanceFromMean(point, wrapAngles);
  // Return null probability if given
  // point is outside angle range
  if (delta.size() == 0)
  {
    return 0.0;
  }

  // Compute gaussian probability
  double tmp1 = -0.5 * delta.transpose() * _covarianceInv * delta;
  double tmp2 = pow(2.0 * M_PI, size) * _determinant;
  return std::exp(tmp1) / std::sqrt(tmp2);
}

double GaussianDistribution::logProbability(const Eigen::VectorXd& point, bool wrapAngles) const
{
  size_t size = _mean.size();
  // Compute distance from mean
  Eigen::VectorXd delta = computeDistanceFromMean(point, wrapAngles);
  // Return null probability if given
  // point is outside angle range
  if (delta.size() == 0)
  {
    return -1000.0;
  }

  // Compute log gaussian probability
  double tmp1 = delta.transpose() * _covarianceInv * delta;
  return -0.5 * (std::log(_determinant) + tmp1 + (double)size * std::log(2.0 * M_PI));
}

void GaussianDistribution::fit(const std::vector<Eigen::VectorXd>& data, const Eigen::VectorXi& isCircular)
{
  // Check sizes
  if (data.size() < 2)
  {
    throw std::logic_error("GaussianDistribution not enough data points");
  }
  size_t size = data.front().size();
  for (size_t i = 0; i < data.size(); i++)
  {
    if ((size_t)data[i].size() != size)
    {
      throw std::logic_error("GaussianDistribution invalid data dimension");
    }
  }
  if (isCircular.size() != 0 && (size_t)isCircular.size() != size)
  {
    throw std::logic_error("GaussianDistribution invalid circular dimension");
  }

  // Circular initialization
  if ((size_t)isCircular.size() == size)
  {
    _isCircular = isCircular;
    // Normalization
    for (size_t i = 0; i < (size_t)_isCircular.size(); i++)
    {
      if (_isCircular(i) != 0)
      {
        _isCircular(i) = 1;
        _haveCircular = true;
      }
    }
  }
  else
  {
    _isCircular = Eigen::VectorXi::Zero(size);
    _haveCircular = false;
  }

  // Compute the mean estimation
  if (_haveCircular)
  {
    // If the dimention is circular,
    // the cartsesian mean (sumX, sumY)
    // is computed.
    // Else, only sumX is used.
    Eigen::VectorXd sumX = Eigen::VectorXd::Zero(size);
    Eigen::VectorXd sumY = Eigen::VectorXd::Zero(size);
    for (size_t i = 0; i < data.size(); i++)
    {
      for (size_t j = 0; j < size; j++)
      {
        if (_isCircular(j) == 0)
        {
          sumX(j) += data[i](j);
        }
        else
        {
          sumX(j) += std::cos(data[i](j));
          sumY(j) += std::sin(data[i](j));
        }
      }
    }
    _mean = Eigen::VectorXd::Zero(size);
    for (size_t j = 0; j < size; j++)
    {
      if (_isCircular(j) == 0)
      {
        _mean(j) = (1.0 / (double)data.size()) * sumX(j);
      }
      else
      {
        double meanX = (1.0 / (double)data.size()) * sumX(j);
        double meanY = (1.0 / (double)data.size()) * sumY(j);
        _mean(j) = std::atan2(meanY, meanX);
      }
    }
  }
  else
  {
    // Standard non circular mean
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(size);
    for (size_t i = 0; i < data.size(); i++)
    {
      sum += data[i];
    }
    _mean = (1.0 / (double)data.size()) * sum;
  }

  // Compute the covariance estimation
  Eigen::MatrixXd sum2 = Eigen::MatrixXd::Zero(size, size);
  for (size_t i = 0; i < data.size(); i++)
  {
    Eigen::VectorXd delta = computeDistanceFromMean(data[i], true);
    sum2 += delta * delta.transpose();
  }
  _covariance = (1.0 / (double)(data.size() - 1)) * sum2;

  // Update the Cholesky decomposition
  computeDecomposition();
}

void GaussianDistribution::computeDecomposition()
{
  // Add small epsilon on diagonal according
  // to Rasmussen 2006
  double epsilon = 1e-9;
  size_t size = _mean.size();
  Eigen::LLT<Eigen::MatrixXd> llt(_covariance + epsilon * Eigen::MatrixXd::Identity(size, size));
  _choleskyDecomposition = llt.matrixL();
  // Check the decomposition
  if (llt.info() == Eigen::NumericalIssue)
  {
    throw std::logic_error("GaussianDistribution Cholesky decomposition error");
  }
  // Compute the covariance determinant
  _determinant = 1.0;
  for (size_t i = 0; i < (size_t)_mean.size(); i++)
  {
    _determinant *= _choleskyDecomposition(i, i);
  }
  _determinant = pow(_determinant, 2);
  // Compute the covariance inverse
  _covarianceInv = llt.solve(Eigen::MatrixXd::Identity(size, size));
}

Eigen::VectorXd GaussianDistribution::computeDistanceFromMean(const Eigen::VectorXd& point, bool doBoundAngles) const
{
  size_t size = _mean.size();
  if ((size_t)point.size() != size)
  {
    throw std::logic_error("GaussianDistribution invalid dimension");
  }

  Eigen::VectorXd delta(size);
  if (_haveCircular)
  {
    for (size_t i = 0; i < size; i++)
    {
      if (_isCircular(i) == 0)
      {
        // No circular distance
        delta(i) = point(i) - _mean(i);
      }
      else
      {
        double angle = point(i);
        // Outside of angle bounds
        if (angle < -M_PI || angle > M_PI)
        {
          if (doBoundAngles)
          {
            angle = AngleBound(angle);
          }
          else
          {
            return Eigen::VectorXd();
          }
        }
        // Compute circular distance
        delta(i) = AngleDistance(_mean(i), angle);
      }
    }
  }
  else
  {
    // No circular distance
    delta = point - _mean;
  }

  return delta;
}

}  // namespace Leph
