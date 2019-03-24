#ifndef LEPH_GAUSSIANDISTRIBUTION_HPP
#define LEPH_GAUSSIANDISTRIBUTION_HPP

#include <vector>
#include <random>
#include <Eigen/Dense>

namespace Leph
{
/**
 * GaussianDistribution
 *
 * Multivariate gaussian
 * normal distribution
 * !!!
 * !!! Note that the current implementation
 * !!! is not really the wrapped normal
 * !!! distribution.
 * !!! Probability and fittting methods are
 * !!! wrong the more the circular
 * !!! std deviation is high.
 * !!!
 */
class GaussianDistribution
{
public:
  /**
   * Dummy empty initialization
   */
  GaussianDistribution();

  /**
   * Initialization with mean
   * vector and covariance
   * matrix.
   * If optional isCircular is
   * not empty, each non zero value
   * means that associated dimension
   * is an angle in radian.
   */
  GaussianDistribution(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance,
                       const Eigen::VectorXi& isCircular = Eigen::VectorXi());

  /**
   * Return the gaussian
   * dimentionality
   */
  size_t dimension() const;

  /**
   * Access to internal mean vector
   * and covariance matrix
   */
  const Eigen::VectorXd& mean() const;
  const Eigen::MatrixXd& covariance() const;

  /**
   * Access to circular
   * dimension configuration
   */
  const Eigen::VectorXi& isCircular() const;

  /**
   * Sample a vector from the
   * multivariate gaussian with
   * given random engine
   * If wrapAngles is false, circular
   * dimension in returned point are
   * bounded inside -PI:PI.
   */
  Eigen::VectorXd sample(std::default_random_engine& engine, bool wrapAngles = false) const;

  /**
   * Return the probability of given
   * vector point and current normal law.
   * If wrapAngles is false, circular
   * dimension in given point are assumed
   * to be bounded inside -PI:PI.
   */
  double probability(const Eigen::VectorXd& point, bool wrapAngles = false) const;

  /**
   * Return the log-probability of given
   * vector point and current normal law
   * If wrapAngles is false, circular
   * dimension in given point are assumed
   * to be bounded inside -PI:PI.
   */
  double logProbability(const Eigen::VectorXd& point, bool wrapAngles = false) const;

  /**
   * Compute the classic estimation of gaussian
   * mean and covariance from given data vectors.
   * If optional isCircular is
   * not empty, each non zero value
   * means that associated dimension
   * is an angle in radian.
   */
  void fit(const std::vector<Eigen::VectorXd>& data, const Eigen::VectorXi& isCircular = Eigen::VectorXi());

private:
  /**
   * The mean vector
   */
  Eigen::VectorXd _mean;

  /**
   * The covariance matrix
   * (symetrix definite positive)
   */
  Eigen::MatrixXd _covariance;

  /**
   * Not null integer for each dimension
   * where the represented value is an angle
   * in radian between -PI and PI.
   * haveCircular is true if at least one
   * dimension is an angle.
   */
  Eigen::VectorXi _isCircular;
  bool _haveCircular;

  /**
   * The inverse of covariance matrix computed
   * through cholesky decomposition
   */
  Eigen::MatrixXd _covarianceInv;

  /**
   * The left side of the Cholesky
   * decomposition of the covariance matrix
   */
  Eigen::MatrixXd _choleskyDecomposition;

  /**
   * The determinant of the
   * covariance matrix
   */
  double _determinant;

  /**
   * Compute the covariance decomposition
   */
  void computeDecomposition();

  /**
   * Return the signed distance between
   * given point and current mean.
   * Vector size are checked.
   * If doBoundAngles is false,
   * empty vector is returned if one
   * circular dimension is outside
   * angle bounds.
   * If doBoundAngles is true, input
   * angular dimension are bound between
   * -PI and PI.
   */
  Eigen::VectorXd computeDistanceFromMean(const Eigen::VectorXd& point, bool doBoundAngles) const;
};

}  // namespace Leph

#endif
