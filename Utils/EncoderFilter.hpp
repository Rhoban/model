#ifndef LEPH_ENCODERFILTER_HPP
#define LEPH_ENCODERFILTER_HPP

#include <Eigen/Dense>

namespace Leph
{
/**
 * EncoderFilter
 *
 * Implement simple linear Kalman
 * model filter for accurate velocity
 * and acceleration estimation of
 * a raw encoder stream of data.
 */
class EncoderFilter
{
public:
  /**
   * Initialization with optional
   * initial state and variances
   */
  EncoderFilter();
  EncoderFilter(double pos, double vel, double acc, double var);
  EncoderFilter(double pos, double vel, double acc, double posVar, double velVar, double accVar);

  /**
   * Set prediction and observation
   * variance parameters
   * (For prediction, single variance to
   * complete coraviance matrix can be given)
   */
  void setObservationVar(double noise);
  void setPredictionVar(double noise);
  void setPredictionVar(const Eigen::Vector3d& noise);
  void setPredictionVar(const Eigen::Matrix3d& noise);

  /**
   * Set current position, velocity
   * and acceleration state
   */
  void setPos(double val);
  void setVel(double val);
  void setAcc(double val);

  /**
   * Return current position,
   * velocity and acceleration
   * variance.
   * Return complete covariance
   * matrix.
   */
  double posVar() const;
  double velVar() const;
  double accVar() const;
  const Eigen::Matrix3d& var() const;

  /**
   * Return current position,
   * velocity and acceleration
   */
  double pos() const;
  double vel() const;
  double acc() const;

  /**
   * Update the filter with given dt
   * time step.
   * The first version is only given the
   * read position.
   * The second version also take into account
   * desired velocity and acceleration.
   */
  void update(double dt, double measurementPos);
  void update(double dt, double measurementPos, double goalVel, double goalAcc);

private:
  /**
   * State (position,
   * velocity and acceleration)
   */
  Eigen::Vector3d _state;

  /**
   * Covariance matrix of current errors
   */
  Eigen::Matrix3d _error;

  /**
   * Coraviance matrix for model prediction
   * and variance of model observation
   */
  Eigen::Matrix3d _predictionNoise;
  double _observationNoise;

  /**
   * Compute the Kalman filtering
   */
  void doKalmanFiltering(const Eigen::Matrix3d& predictionModel, const Eigen::Matrix3d& controlModel,
                         const Eigen::Matrix<double, 1, 3>& observationModel, double measurement,
                         const Eigen::Vector3d& control);
};

}  // namespace Leph

#endif
